/*
 * Copyright 2019-2024 u-blox
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** @file
 * @brief Implementation of the port I2C API for the ESP-IDF platform.
 */

#include "stddef.h"
#include "stdint.h"
#include "stdbool.h"

#include "u_cfg_sw.h"
#include "u_compiler.h" // U_ATOMIC_XXX() macros

#include "u_error_common.h"

#include "u_port.h"
#include "u_port_debug.h"
#include "u_port_os.h"
#include "u_port_i2c.h"

// The legacy I2C driver (driver/i2c.h) is end-of-life from ESP-IDF v6
// and cannot be linked together with the new one, hence the new
// I2C master driver is used here.  Since this API passes the address
// with each transfer, a single device with no address
// (I2C_DEVICE_ADDRESS_NOT_USED) is attached to each bus and the
// address bytes are sent explicitly, as with the legacy command links.
#include "driver/i2c_master.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

#ifndef U_PORT_I2C_MAX_NUM
/** The number of I2C HW blocks that are available on ESP32.
 */
# define U_PORT_I2C_MAX_NUM 2
#endif

/** The read/write bit appended to an address.
 */
#define U_PORT_I2C_READ_BIT  0x01
#define U_PORT_I2C_WRITE_BIT 0x00

/** Make a 7-bit address with a read bit.
 */
#define U_PORT_7BIT_ADDRESS_READ(__ADDRESS__) (((__ADDRESS__) << 1) | U_PORT_I2C_READ_BIT)

/** Make a 7-bit address with a write bit.
 */
#define U_PORT_7BIT_ADDRESS_WRITE(__ADDRESS__) (((__ADDRESS__) << 1) | U_PORT_I2C_WRITE_BIT)

/** Create a header to indicate 10-bit address transmission with a read bit.
 */
#define U_PORT_10BIT_HEADER_READ(__ADDRESS__) ((((__ADDRESS__) & (0x0300)) >> 7) | 0xF0 | U_PORT_I2C_READ_BIT)

/** Create a header to indicate 10-bit address transmission with a write bit.
 */
#define U_PORT_10BIT_HEADER_WRITE(__ADDRESS__) ((((__ADDRESS__) & (0x0300)) >> 7) | 0xF0 | U_PORT_I2C_WRITE_BIT)

/** Get the portion of a 10 bit address that will be sent first (which
 * is the same whether reading or writing).
 */
#define U_PORT_10BIT_ADDRESS(__ADDRESS__) ((__ADDRESS__) & 0xFF)

/** The maximum number of operations in one transfer: START, address
 * (2 bytes for 10-bit), START, address, READ, READ, STOP.
 */
#define U_PORT_I2C_MAX_OPERATIONS 7

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/** Structure of the things we need to keep track of per I2C instance.
 */
typedef struct {
    int32_t pinSda;
    int32_t pinSdc;
    int32_t clockHertz; // This also used as a flag to indicate "in use"
    int32_t timeoutMs;
    bool adopted;
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t device;
} uPortI2cData_t;

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

/** Mutex to ensure thread-safety.
 */
static uPortMutexHandle_t gMutex = NULL;

/** I2C device data.
 */
static uPortI2cData_t gI2cData[U_PORT_I2C_MAX_NUM];

/** Variable to keep track of the number of I2C interfaces open.
 */
static volatile int32_t gResourceAllocCount = 0;

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

// (Re)attach the address-less device, which carries the clock and
// timeout settings, to the bus of an I2C instance.
static esp_err_t attachDevice(int32_t index, int32_t clockHertz,
                              int32_t timeoutMs)
{
    esp_err_t espErr = ESP_OK;
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_DEVICE_ADDRESS_NOT_USED,
        .scl_speed_hz = clockHertz,
        .scl_wait_us = timeoutMs * 1000
    };

    if (gI2cData[index].device != NULL) {
        espErr = i2c_master_bus_rm_device(gI2cData[index].device);
        gI2cData[index].device = NULL;
    }
    if (espErr == ESP_OK) {
        espErr = i2c_master_bus_add_device(gI2cData[index].bus, &cfg,
                                           &gI2cData[index].device);
    }

    return espErr;
}

// Close an I2C instance.
static void closeI2c(int32_t index)
{
    if (gI2cData[index].clockHertz > 0) {
        if (gI2cData[index].device != NULL) {
            i2c_master_bus_rm_device(gI2cData[index].device);
            gI2cData[index].device = NULL;
        }
        if (!gI2cData[index].adopted) {
            i2c_del_master_bus(gI2cData[index].bus);
        }
        gI2cData[index].bus = NULL;
        gI2cData[index].clockHertz = -1;
        U_ATOMIC_DECREMENT(&gResourceAllocCount);
    }
}

// Send an I2C message, returning zero on success else negative error code.
static int32_t send(int32_t handle, uint16_t address,
                    const char *pData, size_t size, bool noStop)
{
    int32_t errorCode = (int32_t) U_ERROR_COMMON_PLATFORM;
    i2c_operation_job_t ops[U_PORT_I2C_MAX_OPERATIONS] = {0};
    uint8_t addressBytes[2];
    size_t numOps = 0;

    ops[numOps++].command = I2C_MASTER_CMD_START;
    // First set up the address
    if (address > 127) {
        addressBytes[0] = U_PORT_10BIT_HEADER_WRITE(address);
        addressBytes[1] = U_PORT_10BIT_ADDRESS(address);
        ops[numOps].write.total_bytes = 2;
    } else {
        addressBytes[0] = U_PORT_7BIT_ADDRESS_WRITE(address);
        ops[numOps].write.total_bytes = 1;
    }
    ops[numOps].command = I2C_MASTER_CMD_WRITE;
    ops[numOps].write.ack_check = true;
    ops[numOps++].write.data = addressBytes;
    // Now add the data, with optional stop marker, and execute it
    if ((pData != NULL) && (size > 0)) {
        ops[numOps].command = I2C_MASTER_CMD_WRITE;
        ops[numOps].write.ack_check = true;
        ops[numOps].write.data = (const uint8_t *) pData;
        ops[numOps++].write.total_bytes = size;
    }
    if (!noStop) {
        ops[numOps++].command = I2C_MASTER_CMD_STOP;
    }
    if (i2c_master_execute_defined_operations(gI2cData[handle].device,
                                              ops, numOps, -1) == ESP_OK) {
        errorCode = (int32_t) U_ERROR_COMMON_SUCCESS;
    }

    return errorCode;
}

// Receive an I2C message, returning number of bytes received on success else
// negative error code.
static int32_t receive(int32_t handle, uint16_t address, char *pData, size_t size)
{
    int32_t errorCodeOrLength = (int32_t) U_ERROR_COMMON_PLATFORM;
    i2c_operation_job_t ops[U_PORT_I2C_MAX_OPERATIONS] = {0};
    uint8_t addressBytes[3];
    size_t numOps = 0;

    ops[numOps++].command = I2C_MASTER_CMD_START;
    // First set up the address
    if (address > 127) {
        addressBytes[0] = U_PORT_10BIT_HEADER_WRITE(address);
        addressBytes[1] = U_PORT_10BIT_ADDRESS(address);
        addressBytes[2] = U_PORT_10BIT_HEADER_READ(address);
        ops[numOps].command = I2C_MASTER_CMD_WRITE;
        ops[numOps].write.ack_check = true;
        ops[numOps].write.data = addressBytes;
        ops[numOps++].write.total_bytes = 2;
        ops[numOps++].command = I2C_MASTER_CMD_START;
        ops[numOps].command = I2C_MASTER_CMD_WRITE;
        ops[numOps].write.ack_check = true;
        ops[numOps].write.data = addressBytes + 2;
        ops[numOps++].write.total_bytes = 1;
    } else {
        addressBytes[0] = U_PORT_7BIT_ADDRESS_READ(address);
        ops[numOps].command = I2C_MASTER_CMD_WRITE;
        ops[numOps].write.ack_check = true;
        ops[numOps].write.data = addressBytes;
        ops[numOps++].write.total_bytes = 1;
    }
    // Now read the data, the last byte with a nack, and execute it
    if (size > 1) {
        ops[numOps].command = I2C_MASTER_CMD_READ;
        ops[numOps].read.ack_value = I2C_ACK_VAL;
        ops[numOps].read.data = (uint8_t *) pData;
        ops[numOps++].read.total_bytes = size - 1;
    }
    if (size > 0) {
        ops[numOps].command = I2C_MASTER_CMD_READ;
        ops[numOps].read.ack_value = I2C_NACK_VAL;
        ops[numOps].read.data = (uint8_t *) (pData + size - 1);
        ops[numOps++].read.total_bytes = 1;
    }
    ops[numOps++].command = I2C_MASTER_CMD_STOP;
    if (i2c_master_execute_defined_operations(gI2cData[handle].device,
                                              ops, numOps, -1) == ESP_OK) {
        errorCodeOrLength = (int32_t) size;
    }

    return errorCodeOrLength;
}

// Open an I2C instance; unlike the other static functions
// this does all the mutex locking etc.
static int32_t openI2c(int32_t i2c, int32_t pinSda, int32_t pinSdc,
                       bool controller, bool adopt)
{
    int32_t handleOrErrorCode = (int32_t) U_ERROR_COMMON_NOT_INITIALISED;
    i2c_master_bus_config_t cfg = {0};
    esp_err_t espErr;

    if (gMutex != NULL) {

        U_PORT_MUTEX_LOCK(gMutex);

        handleOrErrorCode = (int32_t) U_ERROR_COMMON_INVALID_PARAMETER;
        if ((i2c >= 0) && (i2c < sizeof(gI2cData) / sizeof(gI2cData[0])) &&
            (gI2cData[i2c].clockHertz < 0) && controller &&
            (adopt || ((pinSda >= 0) && (pinSdc >= 0)))) {
            handleOrErrorCode = (int32_t) U_ERROR_COMMON_PLATFORM;
            if (adopt) {
                // Use the bus the application has already created
                espErr = i2c_master_get_bus_handle(i2c, &gI2cData[i2c].bus);
            } else {
                cfg.i2c_port = i2c;
                cfg.sda_io_num = pinSda;
                cfg.scl_io_num = pinSdc;
                cfg.clk_source = I2C_CLK_SRC_DEFAULT;
                cfg.glitch_ignore_cnt = 7;
                cfg.flags.enable_internal_pullup = true;
                espErr = i2c_new_master_bus(&cfg, &gI2cData[i2c].bus);
            }
            if (espErr == ESP_OK) {
                gI2cData[i2c].device = NULL;
                espErr = attachDevice(i2c, U_PORT_I2C_CLOCK_FREQUENCY_HERTZ,
                                      U_PORT_I2C_TIMEOUT_MILLISECONDS);
                if (espErr != ESP_OK) {
                    if (!adopt) {
                        i2c_del_master_bus(gI2cData[i2c].bus);
                    }
                    gI2cData[i2c].bus = NULL;
                }
            }
            if (espErr == ESP_OK) {
                gI2cData[i2c].pinSda = pinSda;
                gI2cData[i2c].pinSdc = pinSdc;
                gI2cData[i2c].clockHertz = U_PORT_I2C_CLOCK_FREQUENCY_HERTZ;
                gI2cData[i2c].timeoutMs = U_PORT_I2C_TIMEOUT_MILLISECONDS;
                gI2cData[i2c].adopted = adopt;
                U_ATOMIC_INCREMENT(&gResourceAllocCount);
                // Return the I2C HW block number as the handle
                handleOrErrorCode = i2c;
            }
        }

        U_PORT_MUTEX_UNLOCK(gMutex);
    }

    return handleOrErrorCode;
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * -------------------------------------------------------------- */

// Initialise I2C handling.
int32_t uPortI2cInit()
{
    int32_t errorCode = (int32_t) U_ERROR_COMMON_SUCCESS;

    if (gMutex == NULL) {
        errorCode = uPortMutexCreate(&gMutex);
        if (errorCode == 0) {
            for (size_t x = 0; x < sizeof(gI2cData) / sizeof(gI2cData[0]); x++) {
                gI2cData[x].pinSda = -1;
                gI2cData[x].pinSdc = -1;
                gI2cData[x].clockHertz = -1;
                gI2cData[x].bus = NULL;
                gI2cData[x].device = NULL;
            }
        }
    }

    return errorCode;
}

// Shutdown I2C handling.
void uPortI2cDeinit()
{
    if (gMutex != NULL) {

        U_PORT_MUTEX_LOCK(gMutex);

        // Shut down any open instances
        for (size_t x = 0; x < sizeof(gI2cData) / sizeof(gI2cData[0]); x++) {
            closeI2c(x);
        }

        U_PORT_MUTEX_UNLOCK(gMutex);
        uPortMutexDelete(gMutex);
        gMutex = NULL;
    }
}

// Open an I2C instance.
int32_t uPortI2cOpen(int32_t i2c, int32_t pinSda, int32_t pinSdc,
                     bool controller)
{
    return openI2c(i2c, pinSda, pinSdc, controller, false);
}

// Adopt an I2C instance.
int32_t uPortI2cAdopt(int32_t i2c, bool controller)
{
    return openI2c(i2c, -1, -1, controller, true);
}

// Close an I2C instance.
void uPortI2cClose(int32_t handle)
{
    if ((gMutex != NULL) && (handle >= 0) &&
        (handle < sizeof(gI2cData) / sizeof(gI2cData[0]))) {

        U_PORT_MUTEX_LOCK(gMutex);

        closeI2c(handle);

        U_PORT_MUTEX_UNLOCK(gMutex);
    }
}

// Close an I2C instance and attempt to recover the I2C bus.
int32_t uPortI2cCloseRecoverBus(int32_t handle)
{
    int32_t errorCode = (int32_t) U_ERROR_COMMON_NOT_INITIALISED;

    if (gMutex != NULL) {

        U_PORT_MUTEX_LOCK(gMutex);

        errorCode = (int32_t) U_ERROR_COMMON_INVALID_PARAMETER;
        if ((handle >= 0) && (handle < sizeof(gI2cData) / sizeof(gI2cData[0])) &&
            (gI2cData[handle].clockHertz > 0)) {
            errorCode = (int32_t) U_ERROR_COMMON_NOT_SUPPORTED;
            if (!gI2cData[handle].adopted) {
                closeI2c(handle);
                // Nothing to do - bus recovery is done as required
                // on ESP-IDF; return "not supported" to indicate this
                errorCode = (int32_t) U_ERROR_COMMON_NOT_SUPPORTED;
            }
        }

        U_PORT_MUTEX_UNLOCK(gMutex);
    }

    return errorCode;
}

// Set the I2C clock frequency.
int32_t uPortI2cSetClock(int32_t handle, int32_t clockHertz)
{
    int32_t errorCode = (int32_t) U_ERROR_COMMON_NOT_INITIALISED;

    if (gMutex != NULL) {

        U_PORT_MUTEX_LOCK(gMutex);

        errorCode = (int32_t) U_ERROR_COMMON_INVALID_PARAMETER;
        if ((handle >= 0) && (handle < sizeof(gI2cData) / sizeof(gI2cData[0])) &&
            (gI2cData[handle].clockHertz > 0) && (clockHertz > 0)) {
            errorCode = (int32_t) U_ERROR_COMMON_NOT_SUPPORTED;
            if (!gI2cData[handle].adopted) {
                errorCode = (int32_t) U_ERROR_COMMON_PLATFORM;
                // The clock is a property of the device attached to the bus
                if (attachDevice(handle, clockHertz,
                                 gI2cData[handle].timeoutMs) == ESP_OK) {
                    gI2cData[handle].clockHertz = clockHertz;
                    errorCode = (int32_t) U_ERROR_COMMON_SUCCESS;
                }
            }
        }

        U_PORT_MUTEX_UNLOCK(gMutex);
    }

    return errorCode;
}

// Get the I2C clock frequency.
int32_t uPortI2cGetClock(int32_t handle)
{
    int32_t errorCodeOrClock = (int32_t) U_ERROR_COMMON_NOT_INITIALISED;

    if (gMutex != NULL) {

        U_PORT_MUTEX_LOCK(gMutex);

        errorCodeOrClock = (int32_t) U_ERROR_COMMON_INVALID_PARAMETER;
        if ((handle >= 0) && (handle < sizeof(gI2cData) / sizeof(gI2cData[0])) &&
            (gI2cData[handle].clockHertz > 0)) {
            errorCodeOrClock = (int32_t) U_ERROR_COMMON_NOT_SUPPORTED;
            if (!gI2cData[handle].adopted) {
                errorCodeOrClock = gI2cData[handle].clockHertz;
            }
        }

        U_PORT_MUTEX_UNLOCK(gMutex);
    }

    return errorCodeOrClock;
}

// Set the timeout for I2C.
int32_t uPortI2cSetTimeout(int32_t handle, int32_t timeoutMs)
{
    int32_t errorCode = (int32_t) U_ERROR_COMMON_NOT_INITIALISED;

    if (gMutex != NULL) {

        U_PORT_MUTEX_LOCK(gMutex);

        errorCode = (int32_t) U_ERROR_COMMON_INVALID_PARAMETER;
        if ((handle >= 0) && (handle < sizeof(gI2cData) / sizeof(gI2cData[0])) &&
            (gI2cData[handle].clockHertz > 0) && (timeoutMs > 0)) {
            if (!gI2cData[handle].adopted) {
                errorCode = (int32_t) U_ERROR_COMMON_PLATFORM;
                // The timeout is a property of the device attached to the bus
                if (attachDevice(handle, gI2cData[handle].clockHertz,
                                 timeoutMs) == ESP_OK) {
                    gI2cData[handle].timeoutMs = timeoutMs;
                    errorCode = (int32_t) U_ERROR_COMMON_SUCCESS;
                }
            }
        }

        U_PORT_MUTEX_UNLOCK(gMutex);
    }

    return errorCode;
}

// Get the timeout for I2C.
int32_t uPortI2cGetTimeout(int32_t handle)
{
    int32_t errorCodeOrTimeout = (int32_t) U_ERROR_COMMON_NOT_INITIALISED;

    if (gMutex != NULL) {

        U_PORT_MUTEX_LOCK(gMutex);

        errorCodeOrTimeout = (int32_t) U_ERROR_COMMON_INVALID_PARAMETER;
        if ((handle >= 0) && (handle < sizeof(gI2cData) / sizeof(gI2cData[0])) &&
            (gI2cData[handle].clockHertz > 0)) {
            errorCodeOrTimeout = gI2cData[handle].timeoutMs;
        }

        U_PORT_MUTEX_UNLOCK(gMutex);
    }

    return errorCodeOrTimeout;
}

// Send and/or receive over the I2C interface as a controller.
int32_t uPortI2cControllerExchange(int32_t handle, uint16_t address,
                                   const char *pSend, size_t bytesToSend,
                                   char *pReceive, size_t bytesToReceive,
                                   bool noInterveningStop)
{
    int32_t errorCodeOrLength = (int32_t) U_ERROR_COMMON_NOT_INITIALISED;

    if (gMutex != NULL) {

        U_PORT_MUTEX_LOCK(gMutex);

        errorCodeOrLength = (int32_t) U_ERROR_COMMON_INVALID_PARAMETER;
        if ((handle >= 0) && (handle < sizeof(gI2cData) / sizeof(gI2cData[0])) &&
            (gI2cData[handle].clockHertz > 0) &&
            ((pSend != NULL) || (bytesToSend == 0)) &&
            ((pReceive != NULL) || (bytesToReceive == 0))) {
            errorCodeOrLength = (int32_t) U_ERROR_COMMON_SUCCESS;
            if (pSend != NULL) {
                errorCodeOrLength = send(handle, address, pSend, bytesToSend,
                                         noInterveningStop);
            }
            if ((errorCodeOrLength == (int32_t) U_ERROR_COMMON_SUCCESS) &&
                (pReceive != NULL)) {
                errorCodeOrLength = receive(handle, address, pReceive, bytesToReceive);
            }
        }

        U_PORT_MUTEX_UNLOCK(gMutex);
    }

    return errorCodeOrLength;
}

/** \deprecated please use uPortI2cControllerExchange() instead. */
// Send and/or receive over the I2C interface as a controller.
int32_t uPortI2cControllerSendReceive(int32_t handle, uint16_t address,
                                      const char *pSend, size_t bytesToSend,
                                      char *pReceive, size_t bytesToReceive)
{
    int32_t errorCodeOrLength = (int32_t) U_ERROR_COMMON_NOT_INITIALISED;

    if (gMutex != NULL) {

        U_PORT_MUTEX_LOCK(gMutex);

        errorCodeOrLength = (int32_t) U_ERROR_COMMON_INVALID_PARAMETER;
        if ((handle >= 0) && (handle < sizeof(gI2cData) / sizeof(gI2cData[0])) &&
            (gI2cData[handle].clockHertz > 0) &&
            ((pSend != NULL) || (bytesToSend == 0)) &&
            ((pReceive != NULL) || (bytesToReceive == 0))) {
            errorCodeOrLength = (int32_t) U_ERROR_COMMON_SUCCESS;
            if (pSend != NULL) {
                errorCodeOrLength = send(handle, address, pSend, bytesToSend, false);
            }
            if ((errorCodeOrLength == (int32_t) U_ERROR_COMMON_SUCCESS) &&
                (pReceive != NULL)) {
                errorCodeOrLength = receive(handle, address, pReceive, bytesToReceive);
            }
        }

        U_PORT_MUTEX_UNLOCK(gMutex);
    }

    return errorCodeOrLength;
}

/** \deprecated please use uPortI2cControllerExchange() instead. */
// Perform a send over the I2C interface as a controller.
int32_t uPortI2cControllerSend(int32_t handle, uint16_t address,
                               const char *pSend, size_t bytesToSend,
                               bool noStop)
{
    int32_t errorCode = (int32_t) U_ERROR_COMMON_NOT_INITIALISED;

    if (gMutex != NULL) {

        U_PORT_MUTEX_LOCK(gMutex);

        errorCode = (int32_t) U_ERROR_COMMON_INVALID_PARAMETER;
        if ((handle >= 0) && (handle < sizeof(gI2cData) / sizeof(gI2cData[0])) &&
            (gI2cData[handle].clockHertz > 0) &&
            ((pSend != NULL) || (bytesToSend == 0))) {
            errorCode = send(handle, address, pSend, bytesToSend, noStop);
        }

        U_PORT_MUTEX_UNLOCK(gMutex);
    }

    return errorCode;
}

// Get the number of I2C interfaces currently open.
int32_t uPortI2cResourceAllocCount()
{
    return U_ATOMIC_GET(&gResourceAllocCount);
}

// End of file
