/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>

#define IDENTITY_SDK_TOKEN_LEN 48

// Both buffers must be at least 32 bytes. XXXXXX = last 3 WiFi STA MAC octets.
// node_id "homelink-XXXXXX" (lower) is the backend device identifier (used by
// device-token mint/refresh); ble_name "MuseGadget-XXXXXX" (upper) is the
// advertised BLE name.
void identity_init(void);
const char *identity_node_id(void);
const char *identity_ble_name(void);
const char *identity_mac(void);
const char *identity_device_id(void);
// The maker's SDK token: the one stored in NVS, else the build's
// (CONFIG_GADGET_SDK_TOKEN), else NULL. Call after identity_init().
const char *identity_sdk_token(void);
// Whether token looks like an SDK token, the way the build checks one.
bool identity_sdk_token_valid(const char *token);
// Stores a valid token in NVS; it's used from the next boot. False if token
// isn't valid or can't be stored.
bool identity_set_sdk_token(const char *token);
