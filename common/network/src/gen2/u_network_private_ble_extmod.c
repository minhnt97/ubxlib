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

/* Only #includes of u_* and the C standard library are allowed here,
 * no platform stuff and no OS stuff.  Anything required from
 * the platform/OS must be brought in through u_port* to maintain
 * portability.
 */

/** @file
 * @brief Implementation of the BLE  portion of the
 * network API. The contents of this file aren't any more
 * "private" than the other sources files but the associated header
 * file should be private and this is simply named to match.
 */

#ifndef U_CFG_BLE_MODULE_INTERNAL

#ifdef U_CFG_OVERRIDE
# include "u_cfg_override.h" // For a customer's configuration override
#endif

#include "stddef.h"    // NULL, size_t etc.
#include "stdint.h"    // int32_t etc.
#include "stdbool.h"
#include "string.h"

#include "u_cfg_sw.h"
#include "u_port_os.h"
#include "u_port_debug.h"

#include "u_error_common.h"

#include "u_device_shared.h"

#include "u_network_shared.h"

#include "u_at_client.h"

#include "u_short_range_module_type.h"
#include "u_short_range.h"

#include "u_ble_module_type.h"
#include "u_ble.h"
#include "u_ble_cfg.h"
#include "u_ble_sps.h"

#include "u_network.h"
#include "u_network_config_ble.h"
#include "u_network_private_ble.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

// Call-back for status changes.
//lint -esym(818, pParameter) Suppress "could be declared as pointing to const",
// gotta follow function signature
static void statusCallback(int32_t connHandle, char *pAddress,
                           int32_t status, int32_t channel, int32_t mtu,
                           void *pParameter)
{
    uDeviceInstance_t *pInstance = (uDeviceInstance_t *)pParameter;
    uDeviceNetworkData_t *pNetworkData;
    uNetworkStatusCallbackData_t *pStatusCallbackData;
    bool isUp;
    uNetworkStatus_t networkStatus;

    // Note: can't lock the device API here since we may collide
    // with a network up/down call that will have already locked
    // it and then may, internally, be waiting on something to pass
    // up the event queue that we are currently blocking (since
    // the same event queue is used for most things).
    // We rely on the fact that the various network down calls
    // are well behaved and will not pull the rug out from under
    // one of their callbacks.
    if (uDeviceIsValidInstance(pInstance)) {
        pNetworkData = pUNetworkGetNetworkData(pInstance, U_NETWORK_TYPE_BLE);
        if (pNetworkData != NULL) {
            pStatusCallbackData = (uNetworkStatusCallbackData_t *)pNetworkData->pStatusCallbackData;
            if ((pStatusCallbackData != NULL) && (pStatusCallbackData->pCallback)) {
                networkStatus.ble.pAddress = NULL;
                isUp = (status == (int32_t)U_SHORT_RANGE_EVENT_CONNECTED);
                networkStatus.ble.connHandle = connHandle;
                if (isUp) {
                    networkStatus.ble.pAddress = pAddress;
                }
                networkStatus.ble.status = status;
                networkStatus.ble.channel = channel;
                networkStatus.ble.mtu = mtu;
                pStatusCallbackData->pCallback((uDeviceHandle_t)pInstance, U_NETWORK_TYPE_BLE, isUp,
                                               &networkStatus,
                                               pStatusCallbackData->pCallbackParameter);
            }
        }
    }
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS: WORKAROUND FOR LINKER ISSUE
 * -------------------------------------------------------------- */

void uNetworkPrivateBleLink()
{
    //dummy
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * -------------------------------------------------------------- */

// Bring a BLE interface up or take it down.
int32_t uNetworkPrivateChangeStateBle(uDeviceHandle_t devHandle,
                                      const uNetworkCfgBle_t *pCfg,
                                      bool upNotDown)
{
    int32_t errorCode = (int32_t)U_ERROR_COMMON_INVALID_PARAMETER;
    uBleCfg_t bleCfg;

    if ((pCfg != NULL) && (pCfg->version == 0) &&
        (pCfg->type == U_NETWORK_TYPE_BLE)) {
        bleCfg.role = upNotDown ? (uBleCfgRole_t)pCfg->role : U_BLE_CFG_ROLE_DISABLED;
        bleCfg.spsServer = upNotDown ? pCfg->spsServer : false;
        errorCode = uBleCfgConfigure(devHandle, &bleCfg);
    }
    return errorCode;
}

// Set a call-back to be called when the BLE network status changes.
int32_t uNetworkSetStatusCallbackBle(uDeviceHandle_t devHandle)
{
    return uBleSpsSetCallbackConnectionStatus(devHandle, statusCallback, (void *)devHandle);
}

#endif
// End of file
