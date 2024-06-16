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
 * @brief Stubs to allow the Security Credential API to be compiled without
 * cellular; if you call a cellular API function from the source code
 * here you must also include a weak stub for it which will return
 * #U_ERROR_COMMON_NOT_SUPPORTED when cellular is not included in the build.
 */

#ifdef U_CFG_OVERRIDE
# include "u_cfg_override.h" // For a customer's configuration override
#endif

#include "stddef.h"    // NULL, size_t etc.
#include "stdint.h"    // int32_t etc.
#include "stdbool.h"

#include "u_compiler.h" // U_WEAK
#include "u_error_common.h"
#include "u_at_client.h"
#include "u_device.h"
#include "u_cell_module_type.h"
#include "u_cell.h"

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * -------------------------------------------------------------- */

U_WEAK int32_t uCellAtClientHandleGet(uDeviceHandle_t cellHandle,
                                      uAtClientHandle_t *pAtHandle)
{
    (void) cellHandle;
    (void) pAtHandle;
    return (int32_t) U_ERROR_COMMON_NOT_SUPPORTED;
}

// End of file
