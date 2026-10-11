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

#include "sdkconfig.h"

#if CONFIG_HOMEHUB_BLE_COMMANDS

#include "ble_client.h"

#include <ctype.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_uuid.h"

static const char *TAG = "link.ble_client";

// One operation at a time (the caller's worker task); its result lands here.
static SemaphoreHandle_t s_done;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    bool used;
    uint16_t conn;
    uint8_t addr[6];
    uint8_t addr_type;
    uint16_t mtu;
    bool encrypted;
    ble_client_db_t *db;   // PSRAM, NULL until discovered
} conn_slot_t;

static conn_slot_t s_conns[BLE_CLIENT_MAX_CONNS];

static struct {
    int status;            // 0, NimBLE error, or BLE_CLIENT_E*
    uint16_t conn;         // connect result
    // scan
    ble_client_dev_t *devs;
    int max, count;
    uint32_t reports;
    const char *filter;
    // read
    uint8_t *buf;
    size_t cap, len;
    // discovery
    ble_client_db_t *db;
    int dsc_chr;           // characteristic whose descriptors are being found
    // GAP events that may also arrive unasked complete an operation only
    // when it waits for them.
    uint16_t wait_disconnect;   // conn, or BLE_HS_CONN_HANDLE_NONE
    uint16_t wait_enc;          // conn, or BLE_HS_CONN_HANDLE_NONE
} s_op;

static struct {
    bool active;
    uint16_t conn, handle;
    int64_t t0;
    ble_client_notification_t *out;
    int max, count;
    uint32_t dropped;
} s_sub;

// Subscriptions left on (keep): values that arrive between collections wait
// here for the next ble_client_subscribe() on the same characteristic.
#define KEPT_MAX 4
static struct {
    bool used;
    uint16_t conn, handle;
    ble_client_notification_t *buf;   // BLE_CLIENT_MAX_NOTIFY, PSRAM
    int count;
    uint32_t dropped;
} s_kept[KEPT_MAX];

static void store_notification(ble_client_notification_t *n, int64_t t0, uint16_t handle,
                               struct os_mbuf *om, uint16_t len) {
    n->ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
    n->handle = handle;
    n->len = len;
    uint16_t copy = len < BLE_CLIENT_NOTIFY_BYTES ? len : BLE_CLIENT_NOTIFY_BYTES;
    ble_hs_mbuf_to_flat(om, n->data, copy, NULL);
}

static void kept_drop_conn(uint16_t conn) {
    for (int i = 0; i < KEPT_MAX; i++) {
        if (s_kept[i].used && s_kept[i].conn == conn) {
            heap_caps_free(s_kept[i].buf);
            memset(&s_kept[i], 0, sizeof(s_kept[i]));
        }
    }
}

static bool ensure_ready(void) {
    if (!s_done) s_done = xSemaphoreCreateBinary();
    return s_done && ble_hs_synced();
}

static int wait_done(uint32_t timeout_ms) {
    if (xSemaphoreTake(s_done, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return BLE_CLIENT_ETIMEOUT;
    }
    return s_op.status;
}

static void op_begin(void) {
    s_op.wait_disconnect = BLE_HS_CONN_HANDLE_NONE;
    s_op.wait_enc = BLE_HS_CONN_HANDLE_NONE;
    xSemaphoreTake(s_done, 0);
    s_op.status = 0;
}

static conn_slot_t *slot_for(uint16_t conn) {
    for (int i = 0; i < BLE_CLIENT_MAX_CONNS; i++) {
        if (s_conns[i].used && s_conns[i].conn == conn) return &s_conns[i];
    }
    return NULL;
}

static void uuid_from_any(const ble_uuid_any_t *in, ble_client_uuid_t *out) {
    memset(out, 0, sizeof(*out));
    switch (in->u.type) {
    case BLE_UUID_TYPE_16:
        out->type = 16;
        out->value[0] = in->u16.value & 0xff;
        out->value[1] = in->u16.value >> 8;
        break;
    case BLE_UUID_TYPE_32:
        out->type = 32;
        for (int i = 0; i < 4; i++) out->value[i] = (in->u32.value >> (8 * i)) & 0xff;
        break;
    default:
        out->type = 128;
        memcpy(out->value, in->u128.value, 16);
        break;
    }
}

// ---- scan ---------------------------------------------------------------------

static bool name_matches(const char *name, const char *filter) {
    if (!filter || !filter[0]) return true;
    size_t n = strlen(name), f = strlen(filter);
    for (size_t i = 0; i + f <= n; i++) {
        size_t j = 0;
        while (j < f && tolower((unsigned char)name[i + j]) == tolower((unsigned char)filter[j])) j++;
        if (j == f) return true;
    }
    return false;
}

static void add_uuid(ble_client_dev_t *d, const ble_uuid_any_t *u) {
    ble_client_uuid_t x;
    uuid_from_any(u, &x);
    for (int i = 0; i < d->num_uuids; i++) {
        if (d->uuids[i].type == x.type && !memcmp(d->uuids[i].value, x.value, 16)) return;
    }
    if (d->num_uuids < 4) d->uuids[d->num_uuids++] = x;
}

static void merge_adv(ble_client_dev_t *d, const struct ble_hs_adv_fields *f) {
    if (f->name && f->name_len) {
        size_t n = f->name_len < sizeof(d->name) - 1 ? f->name_len : sizeof(d->name) - 1;
        memcpy(d->name, f->name, n);
        d->name[n] = '\0';
    }
    for (int i = 0; i < f->num_uuids16; i++) {
        ble_uuid_any_t u = {.u16 = f->uuids16[i]};
        add_uuid(d, &u);
    }
    for (int i = 0; i < f->num_uuids32; i++) {
        ble_uuid_any_t u = {.u32 = f->uuids32[i]};
        add_uuid(d, &u);
    }
    for (int i = 0; i < f->num_uuids128; i++) {
        ble_uuid_any_t u = {.u128 = f->uuids128[i]};
        add_uuid(d, &u);
    }
    if (f->mfg_data && f->mfg_data_len) {
        d->mfg_len = f->mfg_data_len < sizeof(d->mfg) ? f->mfg_data_len : sizeof(d->mfg);
        memcpy(d->mfg, f->mfg_data, d->mfg_len);
    }
    if (f->tx_pwr_lvl_is_present) {
        d->tx_power = f->tx_pwr_lvl;
        d->has_tx_power = true;
    }
}

static int scan_event(struct ble_gap_event *event, void *arg) {
    (void)arg;
    if (event->type == BLE_GAP_EVENT_DISC_COMPLETE) {
        xSemaphoreGive(s_done);
        return 0;
    }
    if (event->type != BLE_GAP_EVENT_DISC) return 0;
    s_op.reports++;
    struct ble_hs_adv_fields f;
    if (ble_hs_adv_parse_fields(&f, event->disc.data, event->disc.length_data) != 0) {
        memset(&f, 0, sizeof(f));
    }
    int i;
    for (i = 0; i < s_op.count; i++) {
        if (memcmp(s_op.devs[i].addr, event->disc.addr.val, 6) == 0) break;
    }
    ble_client_dev_t tmp;
    ble_client_dev_t *d;
    if (i < s_op.count) {
        d = &s_op.devs[i];
    } else {
        d = &tmp;
        memset(d, 0, sizeof(*d));
        memcpy(d->addr, event->disc.addr.val, 6);
        d->addr_type = event->disc.addr.type;
    }
    d->rssi = event->disc.rssi;
    if (event->disc.event_type == BLE_HCI_ADV_RPT_EVTYPE_ADV_IND
        || event->disc.event_type == BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {
        d->connectable = true;
    }
    merge_adv(d, &f);
    if (d != &tmp) return 0;
    // A new device: keep it if there's room, or if it's stronger than the
    // weakest kept. The name filter needs a name (it may come in the scan
    // response, so a nameless device is kept until the list fills).
    if (s_op.filter && s_op.filter[0] && d->name[0] && !name_matches(d->name, s_op.filter)) {
        return 0;
    }
    if (s_op.count < s_op.max) {
        s_op.devs[s_op.count++] = *d;
        return 0;
    }
    int weakest = -1;
    for (int k = 0; k < s_op.count; k++) {
        if (weakest < 0 || s_op.devs[k].rssi < s_op.devs[weakest].rssi) weakest = k;
    }
    if (weakest >= 0 && s_op.devs[weakest].rssi < d->rssi) s_op.devs[weakest] = *d;
    return 0;
}

int ble_client_scan(uint32_t duration_ms, bool active, const char *name_filter,
                    ble_client_dev_t *out, int max, uint32_t *reports) {
    if (!ensure_ready()) return BLE_CLIENT_ENOTREADY;
    if (!out || max <= 0 || duration_ms < 100 || duration_ms > 30000) return BLE_CLIENT_EINVAL;
    uint8_t own;
    int rc = ble_hs_id_infer_auto(0, &own);
    if (rc != 0) return rc;
    op_begin();
    s_op.devs = out;
    s_op.max = max;
    s_op.count = 0;
    s_op.reports = 0;
    s_op.filter = name_filter;
    // 30 ms of every 60: half the airtime, the rest left to Wi-Fi.
    struct ble_gap_disc_params p = {
        .itvl = BLE_GAP_SCAN_ITVL_MS(60),
        .window = BLE_GAP_SCAN_WIN_MS(30),
        .filter_duplicates = 0,
        .passive = active ? 0 : 1,
    };
    rc = ble_gap_disc(own, (int32_t)duration_ms, &p, scan_event, NULL);
    if (rc != 0) return rc;
    if (wait_done(duration_ms + 2000) == BLE_CLIENT_ETIMEOUT) ble_gap_disc_cancel();
    // Drop nameless devices that a name filter can't have matched.
    int n = 0;
    for (int i = 0; i < s_op.count; i++) {
        if (name_filter && name_filter[0] && !name_matches(out[i].name, name_filter)) continue;
        out[n++] = out[i];
    }
    if (reports) *reports = s_op.reports;
    s_op.devs = NULL;
    return n;
}

// ---- connections ----------------------------------------------------------------

static int conn_event(struct ble_gap_event *event, void *arg);

static int mtu_cb(uint16_t conn, const struct ble_gatt_error *err, uint16_t mtu, void *arg) {
    (void)arg;
    conn_slot_t *s = slot_for(conn);
    if (s && err->status == 0) s->mtu = mtu;
    s_op.status = err->status;
    xSemaphoreGive(s_done);
    return 0;
}

static void fill_info(const conn_slot_t *s, ble_client_conn_info_t *out) {
    out->conn = s->conn;
    memcpy(out->addr, s->addr, 6);
    out->addr_type = s->addr_type;
    out->mtu = s->mtu ? s->mtu : 23;
    out->central = true;
    out->encrypted = s->encrypted;
}

int ble_client_connect(const uint8_t addr[6], uint8_t addr_type, uint32_t timeout_ms,
                       ble_client_conn_info_t *out) {
    if (!ensure_ready()) return BLE_CLIENT_ENOTREADY;
    for (int i = 0; i < BLE_CLIENT_MAX_CONNS; i++) {
        if (s_conns[i].used && memcmp(s_conns[i].addr, addr, 6) == 0) {
            if (out) fill_info(&s_conns[i], out);
            return 0;
        }
    }
    conn_slot_t *slot = NULL;
    for (int i = 0; i < BLE_CLIENT_MAX_CONNS && !slot; i++) {
        if (!s_conns[i].used) slot = &s_conns[i];
    }
    if (!slot) return BLE_CLIENT_EFULL;
    uint8_t own;
    int rc = ble_hs_id_infer_auto(0, &own);
    if (rc != 0) return rc;
    ble_addr_t peer = {.type = addr_type};
    memcpy(peer.val, addr, 6);
    op_begin();
    s_op.conn = BLE_HS_CONN_HANDLE_NONE;
    rc = ble_gap_connect(own, &peer, (int32_t)timeout_ms, NULL, conn_event, NULL);
    if (rc != 0) return rc;
    rc = wait_done(timeout_ms + 2000);
    if (rc == BLE_CLIENT_ETIMEOUT) {
        ble_gap_conn_cancel();
        return rc;
    }
    if (rc != 0) return rc;
    memset(slot, 0, sizeof(*slot));
    slot->used = true;
    slot->conn = s_op.conn;
    memcpy(slot->addr, addr, 6);
    slot->addr_type = addr_type;
    slot->mtu = 23;
    op_begin();
    if (ble_gattc_exchange_mtu(slot->conn, mtu_cb, NULL) == 0) {
        (void)wait_done(5000);   // a peer that refuses keeps 23
    }
    ESP_LOGI(TAG, "connected conn=%u mtu=%u", slot->conn, slot->mtu);
    if (out) fill_info(slot, out);
    return 0;
}

int ble_client_disconnect(uint16_t conn) {
    if (!ensure_ready()) return BLE_CLIENT_ENOTREADY;
    conn_slot_t *s = slot_for(conn);
    if (!s) return BLE_CLIENT_ENOCONN;
    op_begin();
    s_op.wait_disconnect = conn;
    int rc = ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
    if (rc != 0 && rc != BLE_HS_ENOTCONN) return rc;
    if (rc == 0) (void)wait_done(3000);   // conn_event frees the slot
    if (s->used && s->conn == conn) {
        heap_caps_free(s->db);
        memset(s, 0, sizeof(*s));
    }
    return 0;
}

int ble_client_connections(ble_client_conn_info_t *out, int max) {
    int n = 0;
    for (int i = 0; i < BLE_CLIENT_MAX_CONNS && n < max; i++) {
        if (s_conns[i].used) fill_info(&s_conns[i], &out[n++]);
    }
    return n;
}

int ble_client_secure(uint16_t conn, uint32_t timeout_ms) {
    if (!ensure_ready()) return BLE_CLIENT_ENOTREADY;
    conn_slot_t *s = slot_for(conn);
    if (!s) return BLE_CLIENT_ENOCONN;
    if (s->encrypted) return 0;
    op_begin();
    s_op.wait_enc = conn;
    int rc = ble_gap_security_initiate(conn);
    if (rc != 0) return rc;
    return wait_done(timeout_ms);
}

static int conn_event(struct ble_gap_event *event, void *arg) {
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        s_op.status = event->connect.status;
        s_op.conn = event->connect.conn_handle;
        xSemaphoreGive(s_done);
        return 0;
    case BLE_GAP_EVENT_DISCONNECT: {
        uint16_t conn = event->disconnect.conn.conn_handle;
        ESP_LOGI(TAG, "disconnected conn=%u reason=%d", conn, event->disconnect.reason);
        conn_slot_t *s = slot_for(conn);
        if (s) {
            heap_caps_free(s->db);
            memset(s, 0, sizeof(*s));
        }
        portENTER_CRITICAL(&s_lock);
        if (s_sub.active && s_sub.conn == conn) s_sub.active = false;
        portEXIT_CRITICAL(&s_lock);
        kept_drop_conn(conn);
        if (s_op.wait_disconnect == conn || s_op.wait_enc == conn) {
            s_op.status = s_op.wait_enc == conn ? BLE_HS_ENOTCONN : 0;
            xSemaphoreGive(s_done);
        }
        return 0;
    }
    case BLE_GAP_EVENT_MTU: {
        conn_slot_t *s = slot_for(event->mtu.conn_handle);
        if (s) s->mtu = event->mtu.value;
        return 0;
    }
    case BLE_GAP_EVENT_ENC_CHANGE: {
        conn_slot_t *s = slot_for(event->enc_change.conn_handle);
        if (s && event->enc_change.status == 0) s->encrypted = true;
        if (s_op.wait_enc == event->enc_change.conn_handle) {
            s_op.status = event->enc_change.status;
            xSemaphoreGive(s_done);
        }
        return 0;
    }
    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        // The peer lost our bond: forget it and pair again.
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }
    case BLE_GAP_EVENT_NOTIFY_RX: {
        uint16_t conn = event->notify_rx.conn_handle, handle = event->notify_rx.attr_handle;
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        ble_client_notification_t *n = NULL;
        int64_t t0 = 0;
        portENTER_CRITICAL(&s_lock);
        if (s_sub.active && s_sub.conn == conn && s_sub.handle == handle) {
            if (s_sub.count < s_sub.max) n = &s_sub.out[s_sub.count++];
            else s_sub.dropped++;
            t0 = s_sub.t0;
        } else {
            for (int i = 0; i < KEPT_MAX; i++) {
                if (!s_kept[i].used || s_kept[i].conn != conn || s_kept[i].handle != handle) continue;
                if (s_kept[i].count < BLE_CLIENT_MAX_NOTIFY) n = &s_kept[i].buf[s_kept[i].count++];
                else s_kept[i].dropped++;
                break;
            }
        }
        portEXIT_CRITICAL(&s_lock);
        if (n) store_notification(n, t0, handle, event->notify_rx.om, len);
        return 0;
    }
    default:
        return 0;
    }
}

// ---- discovery ----------------------------------------------------------------

static int svc_cb(uint16_t conn, const struct ble_gatt_error *err,
                  const struct ble_gatt_svc *svc, void *arg) {
    (void)conn;
    (void)arg;
    if (err->status == 0 && svc) {
        ble_client_db_t *db = s_op.db;
        if (db->num_svcs < BLE_CLIENT_MAX_SVCS) {
            ble_client_svc_t *s = &db->svcs[db->num_svcs++];
            uuid_from_any(&svc->uuid, &s->uuid);
            s->start = svc->start_handle;
            s->end = svc->end_handle;
        } else {
            db->truncated = true;
        }
        return 0;
    }
    s_op.status = err->status == BLE_HS_EDONE ? 0 : err->status;
    xSemaphoreGive(s_done);
    return 0;
}

static int chr_cb(uint16_t conn, const struct ble_gatt_error *err,
                  const struct ble_gatt_chr *chr, void *arg) {
    (void)conn;
    if (err->status == 0 && chr) {
        ble_client_db_t *db = s_op.db;
        if (db->num_chrs < BLE_CLIENT_MAX_CHRS) {
            ble_client_chr_t *c = &db->chrs[db->num_chrs++];
            memset(c, 0, sizeof(*c));
            uuid_from_any(&chr->uuid, &c->uuid);
            c->def_handle = chr->def_handle;
            c->val_handle = chr->val_handle;
            c->properties = chr->properties;
            c->svc = (uint8_t)(uintptr_t)arg;
        } else {
            db->truncated = true;
        }
        return 0;
    }
    s_op.status = err->status == BLE_HS_EDONE ? 0 : err->status;
    xSemaphoreGive(s_done);
    return 0;
}

static int dsc_cb(uint16_t conn, const struct ble_gatt_error *err, uint16_t chr_val_handle,
                  const struct ble_gatt_dsc *dsc, void *arg) {
    (void)conn;
    (void)chr_val_handle;
    (void)arg;
    if (err->status == 0 && dsc) {
        if (dsc->uuid.u.type == BLE_UUID_TYPE_16
            && dsc->uuid.u16.value == BLE_GATT_DSC_CLT_CFG_UUID16) {
            s_op.db->chrs[s_op.dsc_chr].cccd_handle = dsc->handle;
        }
        return 0;
    }
    s_op.status = err->status == BLE_HS_EDONE ? 0 : err->status;
    xSemaphoreGive(s_done);
    return 0;
}

int ble_client_discover(uint16_t conn, const ble_client_db_t **out) {
    if (!ensure_ready()) return BLE_CLIENT_ENOTREADY;
    conn_slot_t *s = slot_for(conn);
    if (!s) return BLE_CLIENT_ENOCONN;
    if (s->db) {
        *out = s->db;
        return 0;
    }
    ble_client_db_t *db = heap_caps_calloc(1, sizeof(*db), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!db) db = calloc(1, sizeof(*db));
    if (!db) return BLE_CLIENT_ENOMEM;
    s_op.db = db;
    op_begin();
    int rc = ble_gattc_disc_all_svcs(conn, svc_cb, NULL);
    if (rc == 0) rc = wait_done(10000);
    for (int i = 0; rc == 0 && i < db->num_svcs; i++) {
        op_begin();
        rc = ble_gattc_disc_all_chrs(conn, db->svcs[i].start, db->svcs[i].end, chr_cb,
                                     (void *)(uintptr_t)i);
        if (rc == 0) rc = wait_done(10000);
    }
    // Descriptors, for the CCCD a subscription writes: between a value
    // handle and the next characteristic's declaration (or the service end).
    for (int i = 0; rc == 0 && i < db->num_chrs; i++) {
        ble_client_chr_t *c = &db->chrs[i];
        if (!(c->properties & (BLE_GATT_CHR_PROP_NOTIFY | BLE_GATT_CHR_PROP_INDICATE))) continue;
        uint16_t end = db->svcs[c->svc].end;
        if (i + 1 < db->num_chrs && db->chrs[i + 1].svc == c->svc) end = db->chrs[i + 1].def_handle - 1;
        if (end <= c->val_handle) continue;
        op_begin();
        s_op.dsc_chr = i;
        // NimBLE takes the value handle and searches the handles after it.
        int drc = ble_gattc_disc_all_dscs(conn, c->val_handle, end, dsc_cb, NULL);
        if (drc == 0) drc = wait_done(10000);
        // A peer that fails descriptor discovery still has a usable database;
        // subscribing falls back to the handle after the value.
        if (drc != 0) ESP_LOGW(TAG, "descriptors of handle %u: %d", c->val_handle, drc);
        if (drc == BLE_CLIENT_ETIMEOUT || drc == BLE_HS_ENOTCONN) rc = drc;
    }
    s_op.db = NULL;
    if (rc != 0) {
        heap_caps_free(db);
        return rc;
    }
    s->db = db;
    *out = db;
    return 0;
}

static bool uuid_eq(const ble_client_uuid_t *a, const ble_client_uuid_t *b) {
    // Compare in 128-bit form so 0x180F matches 0000180f-0000-1000-8000-00805f9b34fb.
    static const uint8_t base[16] = {0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                                     0x00, 0x10, 0x00, 0x00, 0, 0, 0, 0};
    uint8_t x[16], y[16];
    const ble_client_uuid_t *in[2] = {a, b};
    uint8_t *o[2] = {x, y};
    for (int k = 0; k < 2; k++) {
        if (in[k]->type == 128) {
            memcpy(o[k], in[k]->value, 16);
        } else {
            memcpy(o[k], base, 16);
            o[k][12] = in[k]->value[0];
            o[k][13] = in[k]->value[1];
            o[k][14] = in[k]->type == 32 ? in[k]->value[2] : 0;
            o[k][15] = in[k]->type == 32 ? in[k]->value[3] : 0;
        }
    }
    return memcmp(x, y, 16) == 0;
}

int ble_client_find_chr(uint16_t conn, uint16_t handle, const ble_client_uuid_t *uuid,
                        const ble_client_uuid_t *svc_uuid, ble_client_chr_t *out) {
    const ble_client_db_t *db;
    int rc = ble_client_discover(conn, &db);
    if (rc != 0) return rc;
    for (int i = 0; i < db->num_chrs; i++) {
        const ble_client_chr_t *c = &db->chrs[i];
        if (handle ? c->val_handle != handle : !uuid || !uuid_eq(&c->uuid, uuid)) continue;
        if (!handle && svc_uuid && !uuid_eq(&db->svcs[c->svc].uuid, svc_uuid)) continue;
        *out = *c;
        return 0;
    }
    return BLE_CLIENT_ENOTFOUND;
}

// ---- read, write, subscribe ---------------------------------------------------

static int read_cb(uint16_t conn, const struct ble_gatt_error *err,
                   struct ble_gatt_attr *attr, void *arg) {
    (void)conn;
    (void)arg;
    if (err->status == 0 && attr && attr->om) {
        uint16_t n = OS_MBUF_PKTLEN(attr->om);
        if (s_op.len + n > s_op.cap) n = (uint16_t)(s_op.cap - s_op.len);
        ble_hs_mbuf_to_flat(attr->om, s_op.buf + s_op.len, n, NULL);
        s_op.len += n;
        return s_op.len >= s_op.cap ? 1 : 0;   // non-zero stops a long read
    }
    s_op.status = (err->status == BLE_HS_EDONE || err->status == 0) ? 0 : err->status;
    xSemaphoreGive(s_done);
    return 0;
}

static bool needs_security(int rc) {
    return rc == BLE_HS_ERR_ATT_BASE + BLE_ATT_ERR_INSUFFICIENT_AUTHEN
        || rc == BLE_HS_ERR_ATT_BASE + BLE_ATT_ERR_INSUFFICIENT_ENC;
}

static int do_read(uint16_t conn, uint16_t handle, uint8_t *buf, size_t cap, size_t *len) {
    op_begin();
    s_op.buf = buf;
    s_op.cap = cap;
    s_op.len = 0;
    int rc = ble_gattc_read_long(conn, handle, 0, read_cb, NULL);
    if (rc == 0) rc = wait_done(10000);
    *len = s_op.len;
    return rc;
}

int ble_client_read(uint16_t conn, uint16_t handle, uint8_t *buf, size_t cap, size_t *len) {
    if (!ensure_ready()) return BLE_CLIENT_ENOTREADY;
    if (!slot_for(conn)) return BLE_CLIENT_ENOCONN;
    int rc = do_read(conn, handle, buf, cap, len);
    if (needs_security(rc) && ble_client_secure(conn, 15000) == 0) {
        rc = do_read(conn, handle, buf, cap, len);
    }
    return rc;
}

static int write_cb(uint16_t conn, const struct ble_gatt_error *err,
                    struct ble_gatt_attr *attr, void *arg) {
    (void)conn;
    (void)attr;
    (void)arg;
    s_op.status = err->status;
    xSemaphoreGive(s_done);
    return 0;
}

static int do_write(uint16_t conn, uint16_t handle, const uint8_t *data, size_t len,
                    bool response) {
    conn_slot_t *s = slot_for(conn);
    size_t att_max = (s && s->mtu > 3) ? s->mtu - 3 : 20;
    if (!response) {
        if (len > att_max) return BLE_CLIENT_EINVAL;
        return ble_gattc_write_no_rsp_flat(conn, handle, data, len);
    }
    op_begin();
    int rc;
    if (len <= att_max) {
        rc = ble_gattc_write_flat(conn, handle, data, len, write_cb, NULL);
    } else {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
        if (!om) return BLE_CLIENT_ENOMEM;
        rc = ble_gattc_write_long(conn, handle, 0, om, write_cb, NULL);
    }
    if (rc == 0) rc = wait_done(10000);
    return rc;
}

int ble_client_write(uint16_t conn, uint16_t handle, const uint8_t *data, size_t len,
                     bool response) {
    if (!ensure_ready()) return BLE_CLIENT_ENOTREADY;
    if (!slot_for(conn)) return BLE_CLIENT_ENOCONN;
    int rc = do_write(conn, handle, data, len, response);
    if (needs_security(rc) && ble_client_secure(conn, 15000) == 0) {
        rc = do_write(conn, handle, data, len, response);
    }
    return rc;
}

int ble_client_subscribe(uint16_t conn, const ble_client_chr_t *chr, uint32_t duration_ms,
                         bool keep, ble_client_notification_t *out, int max,
                         uint32_t *dropped) {
    if (!ensure_ready()) return BLE_CLIENT_ENOTREADY;
    if (!slot_for(conn)) return BLE_CLIENT_ENOCONN;
    if (!(chr->properties & (BLE_GATT_CHR_PROP_NOTIFY | BLE_GATT_CHR_PROP_INDICATE))) {
        return BLE_CLIENT_EINVAL;
    }
    // Most peers put the CCCD right after the value; use that if discovery
    // found none.
    uint16_t cccd = chr->cccd_handle ? chr->cccd_handle : chr->val_handle + 1;
    uint8_t on[2] = {(chr->properties & BLE_GATT_CHR_PROP_NOTIFY) ? 0x01 : 0x02, 0x00};
    int kept = -1;
    for (int i = 0; i < KEPT_MAX; i++) {
        if (s_kept[i].used && s_kept[i].conn == conn && s_kept[i].handle == chr->val_handle) kept = i;
    }
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);
    // Values buffered since a kept subscription come first (ms 0: before this call).
    int count = 0;
    uint32_t drop = 0;
    if (kept >= 0) {
        for (int i = 0; i < s_kept[kept].count && count < max; i++) {
            out[count] = s_kept[kept].buf[i];
            out[count++].ms = 0;
        }
        drop = s_kept[kept].dropped + (uint32_t)(s_kept[kept].count > max ? s_kept[kept].count - max : 0);
        s_kept[kept].count = 0;
        s_kept[kept].dropped = 0;
    }
    s_sub.conn = conn;
    s_sub.handle = chr->val_handle;
    s_sub.out = out;
    s_sub.max = max;
    s_sub.count = count;
    s_sub.dropped = drop;
    s_sub.t0 = now;
    s_sub.active = true;
    portEXIT_CRITICAL(&s_lock);
    int rc = kept >= 0 ? 0 : ble_client_write(conn, cccd, on, sizeof(on), true);
    if (rc == 0) {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
    }
    portENTER_CRITICAL(&s_lock);
    s_sub.active = false;
    count = s_sub.count;
    if (dropped) *dropped = s_sub.dropped;
    portEXIT_CRITICAL(&s_lock);
    if (rc != 0) return rc;
    if (!keep) {
        if (slot_for(conn)) {
            static const uint8_t off[2] = {0, 0};
            (void)ble_client_write(conn, cccd, off, sizeof(off), true);
        }
        if (kept >= 0) {
            portENTER_CRITICAL(&s_lock);
            ble_client_notification_t *buf = s_kept[kept].buf;
            memset(&s_kept[kept], 0, sizeof(s_kept[kept]));
            portEXIT_CRITICAL(&s_lock);
            heap_caps_free(buf);
        }
    } else if (kept < 0) {
        ble_client_notification_t *buf =
            heap_caps_calloc(BLE_CLIENT_MAX_NOTIFY, sizeof(*buf), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        for (int i = 0; buf && i < KEPT_MAX; i++) {
            if (s_kept[i].used) continue;
            portENTER_CRITICAL(&s_lock);
            s_kept[i].conn = conn;
            s_kept[i].handle = chr->val_handle;
            s_kept[i].buf = buf;
            s_kept[i].count = 0;
            s_kept[i].dropped = 0;
            s_kept[i].used = true;
            portEXIT_CRITICAL(&s_lock);
            buf = NULL;
        }
        heap_caps_free(buf);   // no free slot: values between calls are dropped
    }
    return count;
}

void ble_client_reset(void) {
    portENTER_CRITICAL(&s_lock);
    s_sub.active = false;
    portEXIT_CRITICAL(&s_lock);
    for (int i = 0; i < KEPT_MAX; i++) {
        heap_caps_free(s_kept[i].buf);
        memset(&s_kept[i], 0, sizeof(s_kept[i]));
    }
    for (int i = 0; i < BLE_CLIENT_MAX_CONNS; i++) {
        heap_caps_free(s_conns[i].db);
        memset(&s_conns[i], 0, sizeof(s_conns[i]));
    }
}

#endif  // CONFIG_HOMEHUB_BLE_COMMANDS
