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

/** @brief This example demonstrates how to bring up a GNSS device
 * that is directly connected to this MCU and then perform a location
 * fix.
 *
 * The choice of module and the choice of platform on which this
 * code runs is made at build time, see the README.md for
 * instructions.
 */

// Bring in all of the ubxlib public header files
#include "ubxlib.h"

// Bring in the application settings
#include "u_cfg_app_platform_specific.h"

#ifndef U_CFG_DISABLE_TEST_AUTOMATION
// This purely for internal u-blox testing
# include "u_cfg_test_platform_specific.h"
#endif

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// For u-blox internal testing only
#ifdef U_PORT_TEST_ASSERT
# define EXAMPLE_FINAL_STATE(x) U_PORT_TEST_ASSERT(x);
#else
# define EXAMPLE_FINAL_STATE(x)
#endif

#ifndef U_PORT_TEST_FUNCTION
# error if you are not using the unit test framework to run this code you must ensure that the platform clocks/RTOS are set up and either define U_PORT_TEST_FUNCTION yourself or replace it as necessary.
#endif

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

// ZEPHYR USERS may prefer to set the device and network
// configuration from their device tree, rather than in this C
// code: see /port/platform/zephyr/README.md for instructions on
// how to do that.

// GNSS configuration.
// Set U_CFG_TEST_GNSS_MODULE_TYPE to your module type,
// chosen from the values in gnss/api/u_gnss_module_type.h
//
// Note that the pin numbers are those of the MCU: if you
// are using an MCU inside a u-blox module the IO pin numbering
// for the module is likely different to that of the MCU: check
// the data sheet for the module to determine the mapping.

#if defined(U_CFG_TEST_GNSS_MODULE_TYPE) && ((U_CFG_APP_GNSS_UART >= 0) || (U_CFG_APP_GNSS_I2C >= 0) || (U_CFG_APP_GNSS_SPI >= 0))
// DEVICE i.e. module/chip configuration: in this case a GNSS
// module connected via UART or I2C or SPI
static const uDeviceCfg_t gDeviceCfg = {
    .deviceType = U_DEVICE_TYPE_GNSS,
    .deviceCfg = {
        .cfgGnss = {
            .moduleType = U_CFG_TEST_GNSS_MODULE_TYPE,
            .pinEnablePower = U_CFG_APP_PIN_GNSS_ENABLE_POWER,
# ifdef U_CFG_APP_PIN_GNSS_DATA_READY
            .pinDataReady = U_CFG_APP_PIN_GNSS_DATA_READY,
#  ifndef U_CFG_APP_GNSS_DEVICE_PIO_DATA_READY
#   error U_CFG_APP_GNSS_DEVICE_PIO_DATA_READY must be defined if U_CFG_APP_PIN_GNSS_DATA_READY is defined
#  else
            .devicePioDataReady = U_CFG_APP_GNSS_DEVICE_PIO_DATA_READY
#  endif
# else
            .pinDataReady = -1
# endif
            // There is an additional field here
            // "i2cAddress", which we do NOT set,
            // we allow the compiler to set it to 0
            // and all will be fine. You may set the
            // field to the I2C address of your GNSS
            // device if you have modified the I2C
            // address of your GNSS device to something
            // other than the default value of 0x42,
            // for example:
            // .i2cAddress = 0x43
        },
    },
# if (U_CFG_APP_GNSS_I2C >= 0)
    .transportType = U_DEVICE_TRANSPORT_TYPE_I2C,
    .transportCfg = {
        .cfgI2c = {
            .i2c = U_CFG_APP_GNSS_I2C,
            .pinSda = U_CFG_APP_PIN_GNSS_SDA,  // Use -1 if on Zephyr or Linux
            .pinScl = U_CFG_APP_PIN_GNSS_SCL   // Use -1 if on Zephyr or Linux
            // There are three additional fields here,
            // "clockHertz", "alreadyOpen" and
            // "maxSegmentSize", which we do not set,
            // we allow the compiler to set them to 0
            // and all will be fine.
            // You may set clockHertz if you want the
            // I2C bus to use a different clock frequency
            // to the default of
            // #U_PORT_I2C_CLOCK_FREQUENCY_HERTZ, for
            // example:
            // .clockHertz = 400000
            // You may set alreadyOpen to true if you
            // are already using this I2C HW block,
            // with the native platform APIs,
            // elsewhere in your application code,
            // and you would like the ubxlib code
            // to use the I2C HW block WITHOUT
            // [re]configuring it, for example:
            // .alreadyOpen = true
            // if alreadyOpen is set to true then
            // pinSda, pinScl and clockHertz will
            // be ignored.
            // You may set maxSegmentSize if the I2C
            // HW you are using has a size limitation
            // (e.g. nRF52832 does); any I2C transfer
            // greater than this size will be split
            // into N transfers smaller than this size.
        },
    },
# elif (U_CFG_APP_GNSS_SPI >= 0)
    .transportType = U_DEVICE_TRANSPORT_TYPE_SPI,
    .transportCfg = {
        .cfgSpi = {
            .spi = U_CFG_APP_GNSS_SPI,
            .pinMosi = U_CFG_APP_PIN_GNSS_SPI_MOSI,  // Use -1 if on Zephyr or Linux
            .pinMiso = U_CFG_APP_PIN_GNSS_SPI_MISO,  // Use -1 if on Zephyr or Linux
            .pinClk = U_CFG_APP_PIN_GNSS_SPI_CLK,    // Use -1 if on Zephyr or Linux
            // Note: Zephyr users may find it more natural to use
            // .device = U_COMMON_SPI_CONTROLLER_DEVICE_INDEX_DEFAULTS(x)
            // instead of the below, where x is the index of a `cs-gpios`
            // entry that has already been defined for this SPI block in
            // their Zephyr device tree.  For instance, if this SPI block
            // in the device tree contained:
            //     cs-gpios = <&gpio0 2 GPIO_ACTIVE_LOW>,
            //                <&gpio1 14 GPIO_ACTIVE_LOW>;
            // then:
            // .device = U_COMMON_SPI_CONTROLLER_DEVICE_INDEX_DEFAULTS(1)
            // would use pin 14 of port GPIO 1 as the chip select.
            .device = U_COMMON_SPI_CONTROLLER_DEVICE_DEFAULTS(U_CFG_APP_PIN_GNSS_SPI_SELECT)
            // There is an additional field here,
            // "maxSegmentSize", which we do not set,
            // we allow the compiler to set it to 0
            // and all will be fine.
            // You may set maxSegmentSize if the SPI
            // HW you are using has a size limitation
            // (e.g. nRF52832 does); any SPI transfer
            // greater than this size will be split
            // into N transfers smaller than this size.
        },
    },
# else
    .transportType = U_DEVICE_TRANSPORT_TYPE_UART,
    .transportCfg = {
        .cfgUart = {
            .uart = U_CFG_APP_GNSS_UART,
            .baudRate = U_GNSS_UART_BAUD_RATE, /* Use 0 to try all possible baud rates
                                                  and find the correct one. */
            .pinTxd = U_CFG_APP_PIN_GNSS_TXD,  // Use -1 if on Zephyr or Linux or Windows
            .pinRxd = U_CFG_APP_PIN_GNSS_RXD,  // Use -1 if on Zephyr or Linux or Windows
            .pinCts = U_CFG_APP_PIN_GNSS_CTS,  // Use -1 if on Zephyr
            .pinRts = U_CFG_APP_PIN_GNSS_RTS,  // Use -1 if on Zephyr
#ifdef U_CFG_APP_UART_PREFIX
            .pPrefix = U_PORT_STRINGIFY_QUOTED(U_CFG_APP_UART_PREFIX) // Relevant for Linux only
#else
            .pPrefix = NULL
#endif
        },
    },
# endif
};
// NETWORK configuration for GNSS
static const uNetworkCfgGnss_t gNetworkCfg = {
    .type = U_NETWORK_TYPE_GNSS,
    .moduleType = U_CFG_TEST_GNSS_MODULE_TYPE,
    .devicePinPwr = -1,
    .devicePinDataReady = -1
};
#else
static const uDeviceCfg_t gDeviceCfg = {.deviceType = U_DEVICE_TYPE_NONE};
static const uNetworkCfgGnss_t gNetworkCfg = {.type = U_NETWORK_TYPE_NONE};
#endif

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

// Convert a lat/long into a whole number and a bit-after-the-decimal-point
// that can be printed by a version of printf() that does not support
// floating point operations, returning the prefix (either "+" or "-").
// The result should be printed with printf() format specifiers
// %c%d.%07d, e.g. something like:
//
// int32_t whole;
// int32_t fraction;
//
// printf("%c%d.%07d/%c%d.%07d", latLongToBits(latitudeX1e7, &whole, &fraction),
//                               whole, fraction,
//                               latLongToBits(longitudeX1e7, &whole, &fraction),
//                               whole, fraction);
static char latLongToBits(int32_t thingX1e7,
                          int32_t *pWhole,
                          int32_t *pFraction)
{
    char prefix = '+';

    // Deal with the sign
    if (thingX1e7 < 0) {
        thingX1e7 = -thingX1e7;
        prefix = '-';
    }
    *pWhole = thingX1e7 / 10000000;
    *pFraction = thingX1e7 % 10000000;

    return prefix;
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS: THE EXAMPLE
 * -------------------------------------------------------------- */

// The entry point, main(): before this is called the system
// clocks must have been started and the RTOS must be running;
// we are in task space.
U_PORT_TEST_FUNCTION("[example]", "exampleLocGnss")
{
    uDeviceHandle_t devHandle = NULL;
    uLocation_t location;
    int32_t whole = 0;
    int32_t fraction = 0;
    int32_t returnCode;

    // Set an out of range value so that we can test it later
    location.timeUtc = -1;

    // Initialise the APIs we will need
    uPortInit();
    uPortI2cInit(); // You only need this if an I2C interface is used
    uPortSpiInit(); // You only need this if an SPI interface is used
    uDeviceInit();

    // Open the device
    returnCode = uDeviceOpen(&gDeviceCfg, &devHandle);
    uPortLog("Opened device with return code %d.\n", returnCode);

    if (returnCode == 0) {
        // You may configure GNSS as required here
        // here using any of the GNSS API calls.

        // Bring up the GNSS network interface
        uPortLog("Bringing up the network...\n");
        if (uNetworkInterfaceUp(devHandle, U_NETWORK_TYPE_GNSS,
                                &gNetworkCfg) == 0) {

            // Get location
            if (uLocationGet(devHandle, U_LOCATION_TYPE_GNSS,
                             NULL, NULL, &location, NULL) == 0) {
                uPortLog("I am here: https://maps.google.com/?q=%c%d.%07d,%c%d.%07d\n",
                         latLongToBits(location.latitudeX1e7, &whole, &fraction),
                         whole, fraction,
                         latLongToBits(location.longitudeX1e7, &whole, &fraction),
                         whole, fraction);
            } else {
                uPortLog("Unable to get a location fix!\n");
            }

            // When finished with the GNSS network layer
            uPortLog("Taking down GNSS...\n");
            uNetworkInterfaceDown(devHandle, U_NETWORK_TYPE_GNSS);
        } else {
            uPortLog("Unable to bring up GNSS!\n");
        }
        // Close the device
        // Note: we don't power the device down here in order
        // to speed up testing; you may prefer to power it off
        // by setting the second parameter to true.
        uDeviceClose(devHandle, false);

    } else {
        uPortLog("Unable to bring up the device!\n");
    }

    // Tidy up
    uDeviceDeinit();
    uPortSpiDeinit(); // You only need this if an SPI interface is used
    uPortI2cDeinit(); // You only need this if an I2C interface is used
    uPortDeinit();

    uPortLog("Done.\n");

#if defined(U_CFG_TEST_GNSS_MODULE_TYPE) && ((U_CFG_APP_GNSS_UART >= 0) || (U_CFG_APP_GNSS_I2C >= 0) || (U_CFG_APP_GNSS_SPI >= 0))
    // For u-blox internal testing only
    EXAMPLE_FINAL_STATE(location.timeUtc > 0);
#endif
}

// End of file
