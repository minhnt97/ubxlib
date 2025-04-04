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

#ifndef _U_GNSS_PRIVATE_H_
#define _U_GNSS_PRIVATE_H_

/* Only header files representing a direct and unavoidable
 * dependency between the API of this module and the API
 * of another module should be included here; otherwise
 * please keep #includes to your .c files. */

#include "u_device.h"
#include "u_ringbuffer.h"
#include "u_gnss_info.h" // For uGnssVersionType_t

/** @file
 * @brief This header file defines types, functions and inclusions that
 * are common and private to the GNSS API.
 *
 * IMPORTANT: the vast majority of these functions are NOT thread-safe
 * as they use the GNSS instance pointer; it is generally up to you
 * to lock the gUGnssPrivateMutex beforehand in order that the instance
 * pointer is protected from modification by another thread.
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

#ifndef U_GNSS_MAX_UBX_PROTOCOL_MESSAGE_BODY_LENGTH_BYTES
/** The maximum size of UBX-format message body to be read using
 * these functions.  The maximum length of an RRLP message
 * (UBX-RXM-MEASX) is the governing factor here.  Note that when
 * using a  streamed transport messages can be of arbitrary
 * length, this limit does not apply.
 */
# define U_GNSS_MAX_UBX_PROTOCOL_MESSAGE_BODY_LENGTH_BYTES 1024
#endif

#ifndef U_GNSS_MSG_RING_BUFFER_LENGTH_BYTES
/** The size of the ring buffer that is used to hold messages
 * streamed (e.g. over I2C or UART or SPI) from the GNSS chip.
 * Should be big enough to hold a few long messages from the device
 * while these are read asynchronously in task-space by the
 * application.
 */
# define U_GNSS_MSG_RING_BUFFER_LENGTH_BYTES 2048
#endif

#ifndef U_GNSS_RING_BUFFER_MAX_FILL_TIME_MS
/** A useful maximum for the amount of time spent pulling
 * data into the ring buffer (for streamed sources such as
 * I2C, UART or SPI).
 */
# define U_GNSS_RING_BUFFER_MAX_FILL_TIME_MS 2000
#endif

#ifndef U_GNSS_RING_BUFFER_MIN_FILL_TIME_MS
/** A useful minimum for the amount of time spent pulling
 * data into the ring buffer (for streamed sources such as
 * I2C, UART or SPI), if you aren't just going to read what's
 * already there (in which case use 0).
 */
# define U_GNSS_RING_BUFFER_MIN_FILL_TIME_MS 100
#endif

/** Determine if the given feature is supported or not
 * by the pointed-to module.
 */
//lint --emacro((774), U_GNSS_PRIVATE_HAS) Suppress left side always
// evaluates to True
//lint -esym(755, U_GNSS_PRIVATE_HAS) Suppress macro not
// referenced it may be conditionally compiled-out.
#define U_GNSS_PRIVATE_HAS(pModule, feature) \
    ((pModule != NULL) && ((pModule->featuresBitmap) & (1UL << (int32_t) (feature))))

/** Flag to indicate that the pos task has run (for synchronisation
 * purposes.
 */
#define U_GNSS_POS_TASK_FLAG_HAS_RUN    0x01

/** Flag to indicate that the pos task should keep waiting
 * for a single position fix.
 */
#define U_GNSS_POS_TASK_FLAG_KEEP_GOING 0x02

/** Flag to indicate that the pos task should call the callback
 * for each position fix in continuous mode.
 */
#define U_GNSS_POS_TASK_FLAG_CONTINUOUS 0x04

/** The value that constitutes "no data" on SPI.
 */
#define U_GNSS_PRIVATE_SPI_FILL 0xFF

#ifndef U_GNSS_CFG_LAYERS_SET
/** The layers to use when using CFG-VAL to set a configuration
 * value: both RAM and BBRAM if on/off power saving might be used
 * since RAM is erased in the power-off state.
 */
# define U_GNSS_CFG_LAYERS_SET (U_GNSS_CFG_VAL_LAYER_RAM | U_GNSS_CFG_VAL_LAYER_BBRAM)
#endif

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/** Features of a module that require different compile-time
 * behaviours in this implementation.
 */
//lint -esym(756, uGnssPrivateFeature_t) Suppress not referenced,
// Lint can't seem to find it inside macros.
typedef enum {
    U_GNSS_PRIVATE_FEATURE_CFGVALXXX,
    U_GNSS_PRIVATE_FEATURE_GEOFENCE,
    U_GNSS_PRIVATE_FEATURE_OLD_CFG_API,
    U_GNSS_PRIVATE_FEATURE_RXM_MEAS_50_20_C12_D12
} uGnssPrivateFeature_t;

/** The characteristics that may differ between GNSS modules.
 * Note: order is important since this is statically initialised.
 */
typedef struct {
    uGnssModuleType_t moduleType; /**< the module type. */
    uint32_t featuresBitmap; /**< a bit-map of the uGnssPrivateFeature_t
                                  characteristics of this module. */
} uGnssPrivateModule_t;

/** The stream types.
 */
typedef enum {
    U_GNSS_PRIVATE_STREAM_TYPE_NONE,
    U_GNSS_PRIVATE_STREAM_TYPE_UART,
    U_GNSS_PRIVATE_STREAM_TYPE_I2C,
    U_GNSS_PRIVATE_STREAM_TYPE_SPI,
    U_GNSS_PRIVATE_STREAM_TYPE_VIRTUAL_SERIAL,
    U_GNSS_PRIVATE_STREAM_TYPE_MAX_NUM
} uGnssPrivateStreamType_t;

/** Enum that maps to the Virtual Pin manager types reported by
 * the UBX-MON-HW3 message.
 *
 * Note: these values are not complete and may not be completely
 * accurate, however they are sufficient to permit the TX-Ready
 * pin to be managed, see the gVirtualPin* arrays in u_gnss_private.c.
 *
 * TODO: would be good to add USB and maybe UART2 to this list.
 */
typedef enum {
    U_GNSS_PRIVATE_VIRTUAL_PIN_NONE = -1,
    U_GNSS_PRIVATE_VIRTUAL_PIN_UART_RXD = 0,
    U_GNSS_PRIVATE_VIRTUAL_PIN_UART_TXD = 1,
    U_GNSS_PRIVATE_VIRTUAL_PIN_I2C_SCL = 2,
    U_GNSS_PRIVATE_VIRTUAL_PIN_I2C_SDA = 3,
    U_GNSS_PRIVATE_VIRTUAL_PIN_SPI_MOSI = 6,
    U_GNSS_PRIVATE_VIRTUAL_PIN_SPI_MISO = 7,
    U_GNSS_PRIVATE_VIRTUAL_PIN_SPI_CLK = 8,
    U_GNSS_PRIVATE_VIRTUAL_PIN_SPI_CS = 9,
    U_GNSS_PRIVATE_VIRTUAL_PIN_TIMEPULSE = 16,
    U_GNSS_PRIVATE_VIRTUAL_PIN_EXTINT = 18
} uGnssPrivateVirtualPinType_t;

/** Structure to hold a message ID where the NMEA field is a buffer rather
 * than a pointer to a string.
 */
typedef struct {
    uGnssProtocol_t type;
    union {
        uint16_t ubx; /**< formed of the message class in the most significant byte
                           and the message ID in the least significant byte; where
                           this is employed for matching you may use
                           #U_GNSS_UBX_MESSAGE_CLASS_ALL in the most significant byte
                           for all classes, #U_GNSS_UBX_MESSAGE_ID_ALL in the least
                           significant byte for all IDs. */
        char nmea[U_GNSS_NMEA_MESSAGE_MATCH_LENGTH_CHARACTERS + 1];  /**< "GPGGA", "GNZDA",
                                                                          etc. guaranteed
                                                                          to be
                                                                          null-terminated. */
        uint16_t rtcm;
    } id;
} uGnssPrivateMessageId_t;

/** Structure to hold the data associated with one non-blocking
 * message read utility function, intended to be used in a
 * linked-list.
 */
typedef struct uGnssPrivateMsgReader_t {
    int32_t handle;
    uGnssPrivateMessageId_t privateMessageId;
    void *pCallback; /**< stored as a void * to avoid having to bring
                          all the types of uGnssMsgReceiveCallback_t
                          into everything. */
    void *pCallbackParam;
    struct uGnssPrivateMsgReader_t *pNext;
} uGnssPrivateMsgReader_t;

/** Structure to hold the data associated with the task running
 * the non-blocking message receive utility functions.
 */
typedef struct {
    int32_t nextHandle;
    uPortTaskHandle_t taskHandle;
    char *pTemporaryBuffer;
    uPortMutexHandle_t taskRunningMutexHandle;
    uPortQueueHandle_t taskExitQueueHandle;
    uPortMutexHandle_t readerMutexHandle;
    int32_t ringBufferReadHandle;
    size_t msgBytesLeftToRead;
    uGnssPrivateMsgReader_t *pReaderList;
} uGnssPrivateMsgReceive_t;

/** Parameters to pass to the streamed position callback.
 */
typedef struct {
    uDeviceHandle_t gnssHandle;
    int32_t asyncHandle;
    void (*pCallback) (uDeviceHandle_t gnssHandle,
                       int32_t errorCode,
                       int32_t latitudeX1e7,
                       int32_t longitudeX1e7,
                       int32_t altitudeMillimetres,
                       int32_t radiusMillimetres,
                       int32_t altitudeUncertaintyMillimetres,
                       int32_t speedMillimetresPerSecond,
                       int32_t svs,
                       int32_t pDopX1e2,
                       int32_t velN, int32_t velE, int32_t velD,
                       int64_t timeUtc);
    int32_t measurementPeriodMs; /**< set to -1 of nothing to restore. */
    int32_t navigationCount;     /**< set to -1 of nothing to restore. */
    int32_t messageRate;         /**< set to -1 of nothing to restore. */
} uGnssPrivateStreamedPosition_t;

/** Parameters for AssistNow.
 */
typedef struct {
    bool (*pProgressCallback)(uDeviceHandle_t, int32_t, size_t, size_t, void *);
    void *pProgressCallbackParam;
    volatile bool transferInProgress;
    size_t blocksTotal;
    int32_t errorCode;
} uGnssPrivateMga_t;

/** Parameters that need to be stored for the MCU-side of
 * Data Ready pin operation.
 */
typedef struct {
    int32_t pinMcu; /**< the pin of the MCU that is connected to the Data Ready (AKA TX-Ready) pin of the GNSS device. */
    bool activeLow; /**< true if a low level on pin indicates that data is ready. */
    int32_t timeoutMs; /**< the time to wait for Data Ready in milliseconds. */
    uPortSemaphoreHandle_t semaphoreHandle;
    void (*pCallback) (uDeviceHandle_t,
                       void *); /**< user callback, called in INTERRUPT CONTEXT when pinDataReady goes active. */
    void *pCallbackParam; /**< optional user parameter passed to pCallback. */
} uGnssPrivateDataReadyMcu_t;

/** Parameters for the device-side of Data Ready pin operation; these
 * don't need to be stored locally, they are configured in the GNSS
 * device and can be read back from the GNSS device.
 */
typedef struct {
    int32_t pio; /**< the PIO of the GNSS device. */
    bool activeLow; /**< true if the PIO should be low when data is ready. */
    size_t thresholdBytes; /**< the threshold at which the data ready indication should be given. */
} uGnssPrivateDataReadyDevice_t;

/** Definition of a GNSS instance.
 * Note: a pointer to this structure is passed to the asynchronous
 * "get position" function (posGetTask()) which does NOT lock the
 * GNSS mutex, hence it is important that no elements that it cares
 * about are modified while it is active (unlikely since it looks
 * at none of note) but, more importantly, posGetTask() is stopped
 * before an instance is removed.
 */
// *INDENT-OFF* (otherwise AStyle makes a mess of this)
typedef struct uGnssPrivateInstance_t {
    uDeviceHandle_t gnssHandle; /**< the handle for this instance. */
    uDeviceHandle_t intermediateHandle; /**< the handle of the device that the GNSS chip is connected via. */
    const uGnssPrivateModule_t *pModule; /**< pointer to the module type. */
    uGnssTransportType_t transportType; /**< the type of transport to use. */
    uGnssTransportHandle_t transportHandle; /**< the handle of the transport to use. */
    uRingBuffer_t *pSpiRingBuffer; /**< local ring buffer needed for SPI data received while we're sending. */
    char *pSpiLinearBuffer; /**< the linear buffer that will be used by pSpiRingBuffer. */
    uRingBuffer_t ringBuffer; /**< the ring buffer where we put messages from the GNSS chip. */
    char *pLinearBuffer; /**< the linear buffer that will be used by ringBuffer. */
    char *pTemporaryBuffer; /**< a temporary buffer, used to get stuff into ringBuffer. */
    int32_t ringBufferReadHandlePrivate; /**< the read handle for this code to use, -1 if there isn't one. */
    int32_t ringBufferReadHandleMsgReceive; /**< the read handle for uGnssUtilTransparentReceive(). */
    uint16_t i2cAddress; /**< the I2C address of the GNSS chip, only relevant if the transport is I2C. */
    int32_t timeoutMs; /**< the timeout for responses from the GNSS chip in milliseconds. */
    int32_t spiFillThreshold; /**< the number of 0xFF fill bytes which constitute "no data" on SPI. */
    bool printUbxMessages; /**< whether debug printing of UBX messages is on or off. */
    int32_t retriesOnNoResponse; /**< number of times to retry message transmission if there is no response. */
    int32_t pinGnssEnablePower; /**< the pin of the MCU that enables power to the GNSS module. */
    int32_t pinGnssEnablePowerOnState; /**< the value to set pinGnssEnablePower to for "on". */
    int32_t atModulePinPwr; /**< the pin of the AT module that enables power to the GNSS chip (only relevant for transport type AT). */
    int32_t atModulePinDataReady; /**< the pin of the AT module that is connected to the Data Ready pin of the GNSS chip (only relevant for transport type AT). */
    uGnssPort_t portNumber; /**< the internal port number of the GNSS device that we are connected on. */
    uPortMutexHandle_t transportMutex; /**< mutex so that we can have an asynchronous
                                            task use the transport. */
    uPortTaskHandle_t posTask; /**< handle for a task associated with
                                    non-blocking position establishment. */
    uPortMutexHandle_t posMutex; /**< handle for mutex associated with
                                      non-blocking position establishment. */
    volatile uint8_t posTaskFlags; /**< flags to synchronisation the pos task. */
    uGnssPrivateMsgReceive_t *pMsgReceive; /**< stuff associated with the asychronous
                                                message receive utility functions. */
    uGnssPrivateStreamedPosition_t *pStreamedPosition; /**< context data for streamed position, hooked
                                                            here so that we can free it */
    uGnssRrlpMode_t rrlpMode; /**< the type of MEASX to use with RRLP capture. */
    uGnssPrivateMga_t *pMga; /**< storage for AssistNow. */
    void *pFenceContext; /**< storage for a uGeofenceContext_t. */
    uGnssPrivateDataReadyMcu_t *pDataReadyMcu; /**< storage for MCU-side Data Ready functionality. */
    struct uGnssPrivateInstance_t *pNext;
} uGnssPrivateInstance_t;
// *INDENT-ON*

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

/** The characteristics of the supported module types, compiled
 * into the driver.
 */
extern const uGnssPrivateModule_t gUGnssPrivateModuleList[];

/** Number of items in the gUGnssPrivateModuleList array.
 */
extern const size_t gUGnssPrivateModuleListSize;

/** Root for the linked list of instances.
 */
extern uGnssPrivateInstance_t *gpUGnssPrivateInstanceList;

/** Mutex to protect the linked list.
 */
extern uPortMutexHandle_t gUGnssPrivateMutex;

/* ----------------------------------------------------------------
 * FUNCTIONS: MISC
 * -------------------------------------------------------------- */

/** Find a GNSS instance in the list by instance handle.  Note
 * that this function accepts any handle from the device API, e.g.
 * if the GNSS network has been brought up on a cellular device then
 * the cellular device handle may be passed in.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param handle  the instance handle.
 * @return        a pointer to the instance.
 */
uGnssPrivateInstance_t *pUGnssPrivateGetInstance(uDeviceHandle_t handle);

/** Get the module characteristics for a given instance.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param gnssHandle  the instance handle.
 * @return            a pointer to the module characteristics.
 */
//lint -esym(714, pUGnssPrivateGetModule) Suppress lack of a reference
//lint -esym(759, pUGnssPrivateGetModule) etc. since use of this function
//lint -esym(765, pUGnssPrivateGetModule) may be compiled-out in various ways
const uGnssPrivateModule_t *pUGnssPrivateGetModule(uDeviceHandle_t gnssHandle);

/** Get the AT handle of the intermediate device.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance  a pointer to the GNSS instance, cannot be NULL.
 * @return               the AT handle of the intermediate device or NULL
 *                       if there is no such device or the handle could not
 *                       be obtained.
 */
uAtClientHandle_t uGnssPrivateGetIntermediateAtHandle(uGnssPrivateInstance_t *pInstance);

/** Send a buffer as hex.
 *
 * @param[in] pBuffer       the buffer to print; cannot be NULL.
 * @param bufferLengthBytes the number of bytes to print.
 */
void uGnssPrivatePrintBuffer(const char *pBuffer, size_t bufferLengthBytes);

/** Get the rate at which position is obtained.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance            a pointer to the GNSS instance, cannot
 *                                 be NULL.
 * @param[in] pMeasurementPeriodMs a place to put the period between
 *                                 measurements in milliseconds; may
 *                                 be NULL.
 * @param[in] pNavigationCount     a place to put the number of measurements
 *                                 that should result in a navigation
 *                                 solution; may be NULL.
 * @param[in] pTimeSystem          a place to put the time system to
 *                                 which measurements are aligned; may
 *                                 be NULL.
 * @return                         the navigation rate in milliseconds; for
 *                                 instance, if the measurement period
 *                                 is one second and the navigation count
 *                                 five then the return value will be 5000,
 *                                 meaning a navigation solution will be
 *                                 made every five seconds.
 */
int32_t uGnssPrivateGetRate(uGnssPrivateInstance_t *pInstance,
                            int32_t *pMeasurementPeriodMs,
                            int32_t *pNavigationCount,
                            uGnssTimeSystem_t *pTimeSystem);

/** Set the rate at which position is obtained.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance        a pointer to the GNSS instance, cannot
 *                             be NULL.
 * @param measurementPeriodMs  the period between measurements in
 *                             milliseconds; specify -1 to leave this
 *                             unchanged.
 * @param navigationCount      the number of measurements that should
 *                             result in a navigation solution; for
 *                             instance, if measurementPeriodMs is 500
 *                             and navigationCount four then a navigation
 *                             solution will result ever 2 seconds.
 *                             Specify -1 to leave this unchanged.
 * @param timeSystem           the time system to which measurements
 *                             are aligned; the value passed in is
 *                             deliberately not range checked so that
 *                             future types unknown to this code
 *                             may be used. Specify -1 to leave this
 *                             unchanged.
 * @return                     zero on success or negative error code.
 */
int32_t uGnssPrivateSetRate(uGnssPrivateInstance_t *pInstance,
                            int32_t measurementPeriodMs,
                            int32_t navigationCount,
                            uGnssTimeSystem_t timeSystem);

/** Get the rate at which a given UBX message ID is emitted on the
 * current transport; this ONLY WORKS FOR M8 AND M9 modules: for
 * M10 modules and later you must find the relevant member from
 * U_GNSS_CFG_VAL_KEY_ITEM_MSGOUT_* in u_gnss_cfg_val_key.h
 * and get the value of that item, e.g.:
 *
 * ```
 * uint32_t keyId = U_GNSS_CFG_VAL_KEY_ITEM_MSGOUT_UBX_NAV_PVT_I2C_U1;
 * uGnssCfgVal_t *pCfgVal = NULL;
 * if (uGnssCfgPrivateValGetListAlloc(pInstance, &keyId, 1,
 *                                    &pCfgVal, U_GNSS_CFG_VAL_LAYER_RAM) == 0);
 *     // The rate is in pCfgVal->value
 *     // Don't forget to free pCfgVal afterwards
 *     uPortFree(pCfgVal);
 * }
 * ```
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance          a pointer to the GNSS instance, cannot
 *                               be NULL.
 * @param[in] pPrivateMessageId  a pointer to the private message ID; cannot
 *                               be NULL and only UBX protocol message
 *                               rates can currently be retrieved this way.
 * @return                       on success the rate (0 for never, 1 for
 *                               once every message, 2 for "emit every
 *                               other message", etc.) else negative
 *                               error code.
 */
int32_t uGnssPrivateGetMsgRate(uGnssPrivateInstance_t *pInstance,
                               uGnssPrivateMessageId_t *pPrivateMessageId);

/** Set the rate at which a given UBX message ID is emitted on the
 * current transport; this ONLY WORKS FOR M8 AND M9 modules: for
 * M10 modules and later you must find the relevant member from
 * U_GNSS_CFG_VAL_KEY_ITEM_MSGOUT_* in u_gnss_cfg_val_key.h
 * and set the value of that item, e.g.:
 *
 * ```
 * uGnssCfgVal_t cfgVal = {U_GNSS_CFG_VAL_KEY_ITEM_MSGOUT_UBX_NAV_PVT_I2C_U1, 1};
 * uGnssCfgPrivateValSetList(pInstance, &cfgVal, 1,
 *                           U_GNSS_CFG_VAL_TRANSACTION_NONE,
 *                           U_GNSS_CFG_VAL_LAYER_RAM);
 * ```
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance          a pointer to the GNSS instance, cannot
 *                               be NULL.
 * @param[in] pPrivateMessageId  a pointer to the private message ID; cannot
 *                               be NULL and only UBX protocol message
 *                               rates can currently be configured this way.
 * @param rate                   the rate: 0 for never, 1 for once every
 *                               message, 2 for "emit every other message",
 *                               etc.
 * @return                       zero on success or negative error code.
 */
int32_t uGnssPrivateSetMsgRate(uGnssPrivateInstance_t *pInstance,
                               uGnssPrivateMessageId_t *pPrivateMessageId,
                               int32_t rate);

/** Get the protocol types output by the GNSS chip; not relevant
 * where an AT transports is in use since only the UBX protocol is
 * currently supported through that transport.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance   a pointer to the GNSS instance, cannot be NULL.
 * @return                a bit-map of the protocol types that are
 *                        being output else negative error code.
 */
int32_t uGnssPrivateGetProtocolOut(uGnssPrivateInstance_t *pInstance);

/** Set the protocol type output by the GNSS chip; not relevant
 * where an AT transports is in use since only the UBX protocol is
 * currently supported through that transport.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance  a pointer to the GNSS instance, cannot be NULL.
 * @param protocol       the protocol type; #U_GNSS_PROTOCOL_ALL may
 *                       be used to enable all of the output protocols
 *                       supported by the GNSS chip (though using this
 *                       with onNotOff set to false will return an error).
 *                       UBX protocol output cannot be switched off
 *                       since it is used by this code.
 * @param onNotOff       whether the given protocol should be on or off.
 * @return               zero on success or negative error code.
 */
int32_t uGnssPrivateSetProtocolOut(uGnssPrivateInstance_t *pInstance,
                                   uGnssProtocol_t protocol,
                                   bool onNotOff);

/** Allocate an interrupt function for use with Data Ready.  If
 * an interrupt function had previously been allocated for the
 * instance then it is re-used with the new pCallback.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance  a pointer to the GNSS instance, cannot be NULL.
 * @param[in] pCallback  the callback that the interrupt function should
 *                       call, cannot be NULL.
 * @return               the interrupt function, else NULL on error.
 */
void (*pUGnssPrivateDataReadyInterruptAlloc(uGnssPrivateInstance_t *pInstance,
                                            void (*pCallback)(uGnssPrivateInstance_t *)))(void);

/** Free an interrupt function that was allocated by
 * pUGnssPrivateDataReadyInterruptAlloc().
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance   a pointer to the GNSS instance, cannot be NULL.
 */
void uGnssPrivateDataReadyInterruptFree(uGnssPrivateInstance_t *pInstance);

/** Get the data ready configuration for the port of the GNSS device
 * we are using.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance         a pointer to the GNSS instance, cannot
 *                              be NULL.
 * @param[out] pDataReadyDevice a pointer to a place to put the Data Ready
 *                              (AKA TX-Ready) configuration, cannot be
 *                              NULL.  If no data ready is set then the
 *                              pio field in the data ready structure
 *                              will be set to -1.
 * @return                      zero on success else negative error code.
 */
int32_t uGnssPrivateGetDataReady(uGnssPrivateInstance_t *pInstance,
                                 uGnssPrivateDataReadyDevice_t *pDataReadyDevice);

/** Set the data ready configuration for the port of the GNSS device
 * we are using.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance        a pointer to the GNSS instance, cannot be
 *                             NULL.
 * @param[in] pDataReadyDevice a pointer to the Data Ready (AKA TX-Ready)
 *                             configuration required, use NULL to switch
 *                             data ready off.
 * @return                     zero on success else negative error code.
 */
int32_t uGnssPrivateSetDataReady(uGnssPrivateInstance_t *pInstance,
                                 uGnssPrivateDataReadyDevice_t *pDataReadyDevice);

/** Wait for the data ready pin to become active.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance  a pointer to the GNSS instance, cannot be NULL.
 * @param timeoutMs      the time to wait in milliseconds.
 * @return               true if the data ready pin is active, else false.
 */
bool uGnssPrivateIsDataReady(uGnssPrivateInstance_t *pInstance,
                             int32_t timeoutMs);

/** Shut-down any Data Ready pin used with a GNSS device.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance  a pointer to the GNSS instance, cannot be NULL.
 */
void uGnssPrivateCleanUpDataReady(uGnssPrivateInstance_t *pInstance);

/** Shut down and free memory from a [potentially] running pos task.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance  a pointer to the GNSS instance, cannot be NULL.
 */
void uGnssPrivateCleanUpPosTask(uGnssPrivateInstance_t *pInstance);

/** Shut down and free memory from streamd position; should be called
 * before uGnssPrivateStopMsgReceive().
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance  a pointer to the GNSS instance, cannot be NULL.
 */
void uGnssPrivateCleanUpStreamedPos(uGnssPrivateInstance_t *pInstance);

/** Check whether a GNSS chip that we are using via a cellular module
 * is on-board the cellular module, in which case the AT+GPIOC
 * comands are not used.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance  a pointer to the GNSS instance, cannot be NULL.
 * @return               true if there is a GNSS chip inside the cellular
 *                       module, else false.
*/
bool uGnssPrivateIsInsideCell(const uGnssPrivateInstance_t *pInstance);

/** Stop the asynchronous message receive task; kept here so that
 * GNSS deinitialisation can call it.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance  a pointer to the GNSS instance, cannot be NULL.
 */
void uGnssPrivateStopMsgReceive(uGnssPrivateInstance_t *pInstance);

/* ----------------------------------------------------------------
 * FUNCTIONS: MESSAGE RELATED
 * -------------------------------------------------------------- */

/** Convert a public message ID to a private message ID.
 *
 * @param[in] pMessageId         the public message ID; cannot be NULL.
 * @param[out] pPrivateMessageId a place to put the private message ID; cannot be NULL.
 * @return                       zero on success else negative error code.
 */
int32_t uGnssPrivateMessageIdToPrivate(const uGnssMessageId_t *pMessageId,
                                       uGnssPrivateMessageId_t *pPrivateMessageId);

/** Convert a private message ID to a public message ID.  Since, for the
 * NMEA case, the public message ID is just a char *, this function MUST
 * be given storage for the NMEA sentence/talker ID in the last parameter.
 *
 * @param[in] pPrivateMessageId  the private message ID; cannot be NULL.
 * @param[out] pNmea             a pointer to a buffer of size at least
 *                               U_GNSS_NMEA_MESSAGE_MATCH_LENGTH_CHARACTERS + 1
 *                               bytes (the +1 for the null terminator) into
 *                               which the NMEA sentence/talker ID of an
 *                               NMEA-type message can be stored: once
 *                               it has been populated the pNmea field of
 *                               pMessageId will be set to point to this
 *                               buffer.  If the message ID type is NMEA
 *                               and pNmea is NULL then this function will
 *                               return an error.
 * @param[in] pMessageId         a place to put the public message ID; cannot
 *                               be NULL.
 * @return                       zero on success else negative error code.
 */
int32_t uGnssPrivateMessageIdToPublic(const uGnssPrivateMessageId_t *pPrivateMessageId,
                                      uGnssMessageId_t *pMessageId,
                                      char *pNmea);

/** Determine if a private message ID is a wanted one.
 *
 * @param[in] pMessageId       the private message ID to check.
 * @param[in] pMessageIdWanted the wanted private message ID.
 *                             pMessageIdWanted, else false.
 * @return                     true if pMessageId is inside
 */
bool uGnssPrivateMessageIdIsWanted(uGnssPrivateMessageId_t *pMessageId,
                                   uGnssPrivateMessageId_t *pMessageIdWanted);

/** Get the various information from the GNSS chip.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance  a pointer to the GNSS instance, cannot be NULL.
 * @param[out] pVer      a pointer to structure where information is copied.
 *                       This pointer cannot be NULL.
 * @return               on sucesss 0, else negative error code.
 */
int32_t uGnssPrivateInfoGetVersions(uGnssPrivateInstance_t *pInstance,
                                    uGnssVersionType_t *pVer);

/* ----------------------------------------------------------------
 * FUNCTIONS: STREAMING TRANSPORT (UART/I2C/VIRTUAL SERIAL) ONLY
 * -------------------------------------------------------------- */

/** Get the private stream type from a given GNSS transport type.
 *
 * @param transportType the GNSS transport type.
 * @return              the private stream type or negative error
 *                      code if transportType is not a streaming
 *                      transport type.
 */
int32_t uGnssPrivateGetStreamType(uGnssTransportType_t transportType);

/** Get the number of bytes waiting for us from the GNSS chip when using
 * a streaming transport (e.g. UART or I2C or SPI or virtual serial).
 *
 * Note: in the case of SPI it is not possible to determine whether
 * there is any data to be received without actually reading it, hence
 * this function does that and stores the data in the internal pSpiRingBuffer
 * from which the caller can extract it.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance  a pointer to the GNSS instance, cannot be NULL.
 * @return              the number of bytes available to be received,
 *                      else negative error code.
 */
int32_t uGnssPrivateStreamGetReceiveSize(uGnssPrivateInstance_t *pInstance);

/** Fill the internal ring buffer with as much data as possible from
 * the GNSS chip when using a streaming transport (e.g. UART or I2C or SPI or
 * virtual serial).
 *
 * Note that the total maximum time that this function might take is
 * timeoutMs + maxTimeMs.  For a "quick check", to just read in a
 * buffer-full of data that is already available, set timeoutMs to 0.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called, but
 * it is also safe to call this from the task that is checking for
 * asynchronous messages, even though that doesn't lock gUGnssPrivateMutex,
 * since it is otherwise thread-safe and that task is brought up and
 * down in a controlled fashion.
 *
 * @param[in] pInstance  a pointer to the GNSS instance, cannot be NULL.
 * @param timeoutMs      how long to wait for data to begin arriving;
 *                       specify 0 for a quick check that will pull
 *                       any existing data into the ring buffer without
 *                       waiting around.
 * @param maxTimeMs      the maximum time to receive for once data has
 *                       begun arriving, basically a guard timer to
 *                       prevent this function blocking for too long;
 *                       if in doubt use #U_GNSS_RING_BUFFER_MAX_FILL_TIME_MS,
 *                       specify 0 for no maximim time; irrelevant if
 *                       timeoutMs is 0.
 * @return               the number of bytes added to the ring buffer,
 *                       else negative error code.
 */
int32_t uGnssPrivateStreamFillRingBuffer(uGnssPrivateInstance_t *pInstance,
                                         int32_t timeoutMs, int32_t maxTimeMs);

/** Examine the given ring buffer, for the given read handle, and determine
 * if it contains the given message ID, or even the sniff of a possibility
 * of it.  If a message header is matched the read pointer for the given
 * handle will be moved up to the start of the message header; if a sniff
 * of a message is found but it is not complete, the pointer will be moved
 * forward somewhat, discarding unwanted data, otherwise the read pointer will be
 * moved on to the write pointer, i.e. the unwanted data that is in the
 * ring buffer will be discarded.  This function does NOT pull any new
 * data into the ring buffer, the caller must call
 * uGnssPrivateStreamFillRingBuffer() to do that, it only parses data
 * that is already in the ring buffer.  See the msgReceiveTask() asynchronous
 * message receive function in u_gnss_msg.c for an example of how this
 * might be done.
 *
 * Note: it is important that pDiscard (see below) is obeyed, i.e.
 * always discard that many bytes of data from the ring-buffer at the
 * given read handle before this function is called again.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called, but
 * it is also safe to call this from the task that is checking for
 * asynchronous messages, even though that doesn't lock gUGnssPrivateMutex,
 * since it is otherwise thread-safe and that task is brought up and
 * down in a controlled fashion.
 *
 * @param[in] pRingBuffer            a pointer to the ring buffer of the
 *                                   GNSS instance, cannot be NULL.
 * @param readHandle                 the read handle of the ring buffer to
                                     read from.
 * @param[in,out] pPrivateMessageId  on entry this should contain the message
 *                                   ID to look for, wild-cards permitted.
 *                                   On return, if a message has been found,
 *                                   this will be populated with the message
 *                                   ID that was found; cannot be NULL.
 * @return                           if the given message ID is detected then
 *                                   the number of bytes of data in it
 *                                   (including $, header, checksum, etc.)
 *                                   will be returned; if the start of a
 *                                   potentially matching message is found
 *                                   but more data is needed to be certain,
 *                                   #U_ERROR_COMMON_TIMEOUT will be returned,
 *                                   else a negative error code will be returned.
 */
int32_t uGnssPrivateStreamDecodeRingBuffer(uRingBuffer_t *pRingBuffer,
                                           int32_t readHandle,
                                           uGnssPrivateMessageId_t *pPrivateMessageId);

/** Read data from the internal ring buffer into the given linear buffer.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called, but
 * it is also safe to call this from the task that is checking for
 * asynchronous receipt of messages, even though that doesn't lock
 * gUGnssPrivateMutex, since it is otherwise thread-safe and that task
 * is brought up and down in a controlled fashion.
 *
 * @param[in] pInstance a pointer to the GNSS instance, cannot be NULL.
 * @param readHandle    the read handle of the ring buffer to read from.
 * @param[in] pBuffer   pointer to a place to put the data; may be NULL
 *                      to throw the data away.
 * @param size          the amount of data to read.
 * @param maxTimeMs     the maximum time to wait for all of the data to
 *                      turn up in milliseconds.
 * @return              the number of bytes copied to pBuffer, else negative
 *                      error code.
 */
int32_t uGnssPrivateStreamReadRingBuffer(uGnssPrivateInstance_t *pInstance,
                                         int32_t readHandle,
                                         char *pBuffer, size_t size,
                                         int32_t maxTimeMs);

/** Take a peek into the internal ring buffer, copying the data into a
 * linear buffer.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called, but
 * it is also safe to call this from the task that is checking for
 * asynchronous recipt of messages, even though that doesn't lock
 * gUGnssPrivateMutex, since it is otherwise thread-safe and that task
 * is brought up and down in a controlled fashion.
 *
 * @param[in] pInstance a pointer to the GNSS instance, cannot be NULL.
 * @param readHandle    the read handle of the ring buffer to read from.
 * @param[in] pBuffer   pointer to a place to put the data; may be NULL
 *                      to throw the data away.
 * @param size          the amount of data to read.
 * @param offset        the offset into pBuffer to begin reading.
 * @param maxTimeMs     the maximum time to wait for all of the data to turn
 *                      up in milliseconds.
 * @return              the number of bytes copied to pBuffer, else negative
 *                      error code.
 */
int32_t uGnssPrivateStreamPeekRingBuffer(uGnssPrivateInstance_t *pInstance,
                                         int32_t readHandle,
                                         char *pBuffer, size_t size,
                                         size_t offset,
                                         int32_t maxTimeMs);

/** Send raw bytes over UART or I2C or SPI or virtual serial; this is
 * exposed specifically for code brough into ubxlib that already
 * encodes full messages (e.g. libMga).
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance a pointer to the GNSS instance, cannot be NULL.
 * @param[in] pBuffer   pointer to the data to write.
 * @param size          the amount of data at pBuffer.
 * @return              the number of bytes sent, else negative error code.
 */
int32_t uGnssPrivateSendOnlyStreamRaw(uGnssPrivateInstance_t *pInstance,
                                      const char *pBuffer, size_t size);

/** Send a UBX format message over UART or I2C or SPI or virtual serial
 * (do not wait for the response).
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance              a pointer to the GNSS instance, cannot
 *                                   be NULL.
 * @param messageClass               the UBX message class to send with.
 * @param messageId                  the UBX message ID to send with.
 * @param[in] pMessageBody           the body of the message to send; may be
 *                                   NULL.
 * @param messageBodyLengthBytes     the amount of data at pMessageBody; must
 *                                   be non-zero if pMessageBody is non-NULL.
 * @return                           the number of bytes sent, INCLUDING
 *                                   UBX protocol coding overhead, else negative
 *                                   error code.
 */
int32_t uGnssPrivateSendOnlyStreamUbxMessage(uGnssPrivateInstance_t *pInstance,
                                             int32_t messageClass,
                                             int32_t messageId,
                                             const char *pMessageBody,
                                             size_t messageBodyLengthBytes);

/** Send a UBX format message that does not have an acknowledgement
 * over a stream and check that it was accepted by the GNSS chip
 * by querying the GNSS chip's message count.  Note that in the case
 * where the GNSS chip is inside or connected via an intermediate
 * (e.g. cellular) module, that module may also be talking to the GNSS
 * chip over the same interface and so, for that case, no additional
 * checking by this "counting" mechanism is done; we have to rely on
 * the transport being good.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance              a pointer to the GNSS instance, cannot
 *                                   be NULL.
 * @param messageClass               the UBX message class to send with.
 * @param messageId                  the UBX message ID to send with.
 * @param[in] pMessageBody           the body of the message to send; may be
 *                                   NULL.
 * @param messageBodyLengthBytes     the amount of data at pMessageBody; must
 *                                   be non-zero if pMessageBody is non-NULL.
 * @return                           the number of bytes sent, INCLUDING
 *                                   UBX protocol coding overhead, else negative
 *                                   error code.
 */
int32_t uGnssPrivateSendOnlyCheckStreamUbxMessage(uGnssPrivateInstance_t *pInstance,
                                                  int32_t messageClass,
                                                  int32_t messageId,
                                                  const char *pMessageBody,
                                                  size_t messageBodyLengthBytes);

/** Wait for the given message, which can be of any type (not just UBX-format)
 * from the GNSS module; the WHOLE message is returned, i.e. header and CRC
 * etc. are included.  This function will internally call
 * uGnssPrivateStreamFillRingBuffer() to fill the ring buffer with data
 * and then uGnssPrivateStreamReadRingBuffer() to read it.
 *
 * Note: if the message ID is set to a particular UBX-format message (i.e.
 * no wild-cards) and a NACK is received for that message then the
 * error code #U_GNSS_ERROR_NACK will be returned (and the message will
 * be discarded).
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance              a pointer to the GNSS instance, cannot
 *                                   be NULL.
 * @param[in,out] pPrivateMessageId  on entry this should contain the
 *                                   message ID to capture, wildcards
 *                                   permitted.  If the message ID is
 *                                   a wildcard then this function will
 *                                   return on the first matching
 *                                   message ID with this field populated
 *                                   with the message ID that was found.
 *                                   Cannot be NULL.
 * @param readHandle                 the read handle.
 * @param[in,out] ppBuffer           a pointer to a pointer to a buffer
 *                                   in which the message will be placed,
 *                                   cannot be NULL.  If ppBuffer points
 *                                   to NULL (i.e *ppBuffer is NULL) then
 *                                   this function will allocate a buffer
 *                                   of the correct size and populate
 *                                   *ppBuffer with the allocated buffer
 *                                   pointer; in this case IT IS UP TO
 *                                   THE CALLER TO uPortFree(*ppBuffer) WHEN
 *                                   DONE.  The entire message, with
 *                                   any header, $, CRC, etc. included,
 *                                   will be written to the buffer.
 * @param size                       the amount of storage at *ppBuffer,
 *                                   zero if ppBuffer points to NULL.
 * @param timeoutMs                  how long to wait for the [first]
 *                                   message to arrive in milliseconds.
 * @param[in] pKeepGoingCallback     a function that will be called
 *                                   while waiting.  As long as
 *                                   pKeepGoingCallback returns true this
 *                                   function will continue to wait until
 *                                   a matching message has arrived or
 *                                   timeoutSeconds have elapsed. If
 *                                   pKeepGoingCallback returns false
 *                                   then this function will return.
 *                                   pKeepGoingCallback can also be used to
 *                                   feed any application watchdog timer that
 *                                   might be running.  May be NULL, in
 *                                   which case this function will wait
 *                                   until the [first] message has arrived
 *                                   or timeoutSeconds have elapsed.
 * @return                           the number of bytes copied into
 *                                   *ppBuffer else negative error code.
 */
int32_t uGnssPrivateReceiveStreamMessage(uGnssPrivateInstance_t *pInstance,
                                         uGnssPrivateMessageId_t *pPrivateMessageId,
                                         int32_t readHandle,
                                         char **ppBuffer, size_t size,
                                         int32_t timeoutMs,
                                         bool (*pKeepGoingCallback)(uDeviceHandle_t gnssHandle));

/** Add received data to the internal SPI buffer.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance a pointer to the GNSS instance, cannot be NULL.
 * @param[in] pBuffer   pointer to the data to be added.
 * @param size          the amount of data at pBuffer.
 * @return              the amount of data now available in the internal
 *                      SPI buffer, else negative error code; note that
 *                      this may be less than "size" bytes if a GNSS SPI
 *                      fill threshold is in use (see
 *                      uGnssSetSpiFillThreshold()).
 */
int32_t uGnssPrivateSpiAddReceivedData(uGnssPrivateInstance_t *pInstance,
                                       const char *pBuffer, size_t size);

/* ----------------------------------------------------------------
 * FUNCTIONS: ANY TRANSPORT
 * -------------------------------------------------------------- */

/** Send a UBX format message to the GNSS module and, optionally, receive
 * a response of known length.  If the message only illicites a simple
 * Ack/Nack from the module then uGnssPrivateSendUbxMessage() must be used
 * instead.  If the response is of unknown length
 * uGnssPrivateSendReceiveUbxMessageAlloc() may be used instead.  Nay be
 * used with any transport.  For a streamed transport this function will
 * internally call uGnssPrivateStreamFillRingBuffer() to fill the ring
 * buffer with data and then uGnssPrivateStreamReadRingBuffer() to read it.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance              a pointer to the GNSS instance, cannot
 *                                   be NULL.
 * @param messageClass               the UBX message class.
 * @param messageId                  the UBX message ID.
 * @param[in] pMessageBody           the body of the message to send; may be
 *                                   NULL.
 * @param messageBodyLengthBytes     the amount of data at pMessageBody; must
 *                                   be non-zero if pMessageBody is non-NULL.
 * @param[out] pResponseBody         a pointer to somewhere to store the
 *                                   response body, if one is expected; may
 *                                   be NULL.
 * @param maxResponseBodyLengthBytes the amount of storage at pResponseBody;
 *                                   must be non-zero if pResponseBody is non-NULL.
 * @return                           the number of bytes in the body of the response
 *                                   from the GNSS module (irrespective of the value
 *                                   of maxResponseBodyLengthBytes), else negative
 *                                   error code.
 */
int32_t uGnssPrivateSendReceiveUbxMessage(uGnssPrivateInstance_t *pInstance,
                                          int32_t messageClass,
                                          int32_t messageId,
                                          const char *pMessageBody,
                                          size_t messageBodyLengthBytes,
                                          char *pResponseBody,
                                          size_t maxResponseBodyLengthBytes);

/** Send a UBX format message to the GNSS module and receive a response of
 * unknown length, allocating memory to do so. IT IS UP TO THE CALLER TO
 * FREE THIS MEMORY WHEN DONE.  May be used with any transport.  For a
 * streamed transport this function will internally call
 * uGnssPrivateStreamFillRingBuffer() to fill the ring buffer with data
 * and then uGnssPrivateStreamReadRingBuffer() to read it.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance              a pointer to the GNSS instance, cannot
 *                                   be NULL.
 * @param messageClass               the UBX message class.
 * @param messageId                  the UBX message ID.
 * @param[in] pMessageBody           the body of the message to send; may be
 *                                   NULL.
 * @param messageBodyLengthBytes     the amount of data at pMessageBody; must
 *                                   be non-zero if pMessageBody is non-NULL.
 * @param[out] ppResponseBody        a pointer to a pointer that will be
 *                                   populated with the allocated memory
 *                                   containing the body of the response.
 *                                   Cannot be NULL.
 * @return                           the number of bytes of data at
 *                                   ppResponseBody, else negative error code.
 */
int32_t uGnssPrivateSendReceiveUbxMessageAlloc(uGnssPrivateInstance_t *pInstance,
                                               int32_t messageClass,
                                               int32_t messageId,
                                               const char *pMessageBody,
                                               size_t messageBodyLengthBytes,
                                               char **ppResponseBody);

/** Send a UBX format message to the GNSS module that only has an Ack
 * response and check that it is Acked.  May be used with any transport.
 *
 * Note: gUGnssPrivateMutex should be locked before this is called.
 *
 * @param[in] pInstance              a pointer to the GNSS instance, cannot
 *                                   be NULL.
 * @param messageClass               the UBX message class.
 * @param messageId                  the UBX message ID.
 * @param[in] pMessageBody           the body of the message to send; may be
 *                                   NULL.
 * @param messageBodyLengthBytes     the amount of data at pMessageBody; must
 *                                   be non-zero if pMessageBody is non-NULL.
 * @return                           zero on success else negative error code;
 *                                   if the message has been nacked by the GNSS
 *                                   module #U_GNSS_ERROR_NACK will be returned.
 */
int32_t uGnssPrivateSendUbxMessage(uGnssPrivateInstance_t *pInstance,
                                   int32_t messageClass,
                                   int32_t messageId,
                                   const char *pMessageBody,
                                   size_t messageBodyLengthBytes);

#ifdef __cplusplus
}
#endif

#endif // _U_GNSS_PRIVATE_H_

// End of file
