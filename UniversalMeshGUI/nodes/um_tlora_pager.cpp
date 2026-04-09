#include <Arduino.h>
#include <ArduinoJson.h>
#include <LilyGoLib.h>
#include <LV_Helper.h>
#include <lvgl.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "UniversalMesh.h"
#include "ota_update.h"

// -------------------------------------------------------
// Config
// -------------------------------------------------------
#ifndef NODE_NAME
  #define NODE_NAME "um-pager"
#endif
#define UM_HB_INTERVAL   120000UL
#define UM_TEMP_INTERVAL 180000UL
#define UM_LOG_ROWS      20
#define UM_FULL_LEN      320

// -------------------------------------------------------
// Mesh state
// -------------------------------------------------------
enum UMState { UM_DISCOVERING, UM_CONNECTED, UM_NO_COORD };

static UniversalMesh     um_mesh;
static volatile UMState  um_state        = UM_DISCOVERING;
static volatile bool     um_otaRequested = false;
static uint8_t           um_myMac[6]     = {};
static uint8_t           um_coordMac[6]  = {};
static uint8_t           um_channel      = 0;
static unsigned long     um_lastHB       = 0;
static unsigned long     um_lastTemp     = 0;
static TaskHandle_t      um_task         = NULL;
static SemaphoreHandle_t um_mutex        = NULL;
static char    um_log[UM_LOG_ROWS][72]               = {};
static char    um_log_full[UM_LOG_ROWS][UM_FULL_LEN] = {};
static uint8_t um_logHead                            = 0;
static uint8_t um_logCount                           = 0;
static bool    um_log_dirty                          = false;

// -------------------------------------------------------
// LVGL state
// -------------------------------------------------------
static lv_timer_t *um_timer       = NULL;
static lv_timer_t *um_boot_timer  = NULL;
static lv_obj_t   *um_root        = NULL;
static lv_obj_t   *um_boot_cont   = NULL;
static lv_obj_t   *um_status_lbl  = NULL;
static lv_obj_t   *um_info_lbl    = NULL;
static lv_obj_t   *um_log_cont    = NULL;
static lv_obj_t   *um_detail_cont = NULL;
static uint8_t     um_dotPhase    = 0;

// -------------------------------------------------------
// Log helpers
// -------------------------------------------------------
static void um_log_push(const char *line, const char *full = nullptr)
{
    if (!um_mutex) return;
    if (xSemaphoreTake(um_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    strncpy(um_log[um_logHead], line, 71);
    um_log[um_logHead][71] = '\0';
    const char *src = full ? full : line;
    strncpy(um_log_full[um_logHead], src, UM_FULL_LEN - 1);
    um_log_full[um_logHead][UM_FULL_LEN - 1] = '\0';
    um_logHead = (um_logHead + 1) % UM_LOG_ROWS;
    if (um_logCount < UM_LOG_ROWS) um_logCount++;
    um_log_dirty = true;
    xSemaphoreGive(um_mutex);
}

static const char *um_type_name(uint8_t t)
{
    switch (t) {
        case 0x12: return "PING";
        case 0x13: return "PONG";
        case 0x14: return "ACK";
        case 0x15: return "DATA";
        case 0x16: return "KEYREQ";
        case 0x17: return "SEC";
        case 0x18: return "E2E";
        default:   return "???";
    }
}

// -------------------------------------------------------
// sendInfo — JSON node info reply
// -------------------------------------------------------
static void um_sendInfo(uint8_t *destMac, uint8_t appId)
{
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             um_myMac[0], um_myMac[1], um_myMac[2],
             um_myMac[3], um_myMac[4], um_myMac[5]);
    JsonDocument doc;
    doc["n"]    = NODE_NAME;
    doc["mac"]  = macStr;
    doc["up"]   = (unsigned long)(millis() / 1000UL);
    doc["heap"] = ESP.getFreeHeap();
    doc["rssi"] = WiFi.RSSI();
    doc["ch"]   = um_channel;
    doc["chip"] = ESP.getChipModel();
    doc["rev"]  = (int)ESP.getChipRevision();
    String out;
    serializeJson(doc, out);
    um_mesh.send(destMac, MESH_TYPE_DATA, appId, out);
}

// -------------------------------------------------------
// Mesh receive callback — log to UI + handle commands
// -------------------------------------------------------
static void um_on_receive(MeshPacket *pkt, uint8_t *senderMac)
{
    // Build short display line
    char line[72];
    uint8_t slen = pkt->payloadLen > 36 ? 36 : pkt->payloadLen;
    char snippet[37] = {};
    for (int i = 0; i < slen; i++) {
        uint8_t b = pkt->payload[i];
        snippet[i] = (b >= 0x20 && b < 0x7F) ? (char)b : '.';
    }
    snprintf(line, sizeof(line), "%02X:%02X %s A%02X %s",
             senderMac[4], senderMac[5],
             um_type_name(pkt->type), pkt->appId, snippet);

    // Build full detail text
    char full[UM_FULL_LEN];
    uint8_t plen = pkt->payloadLen > 200 ? 200 : pkt->payloadLen;
    char payload[201] = {};
    for (int i = 0; i < plen; i++) {
        uint8_t b = pkt->payload[i];
        payload[i] = (b >= 0x20 && b < 0x7F) ? (char)b : '.';
    }
    snprintf(full, sizeof(full),
             "From: %02X:%02X:%02X:%02X:%02X:%02X\n"
             "To:   %02X:%02X:%02X:%02X:%02X:%02X\n"
             "Type: %s (0x%02X)\n"
             "AppID: 0x%02X\n\n"
             "%s",
             senderMac[0], senderMac[1], senderMac[2],
             senderMac[3], senderMac[4], senderMac[5],
             pkt->destMac[0], pkt->destMac[1], pkt->destMac[2],
             pkt->destMac[3], pkt->destMac[4], pkt->destMac[5],
             um_type_name(pkt->type), pkt->type,
             pkt->appId, payload);
    um_log_push(line, full);

    // Command handling — DATA packets addressed to us
    bool directToMe = (memcmp(pkt->destMac, um_myMac, 6) == 0);
    if (pkt->type != MESH_TYPE_DATA || !directToMe) return;
    if (plen < 4 || strncmp(payload, "cmd:", 4) != 0) return;

    bool fromCoord = (memcmp(senderMac, um_coordMac, 6) == 0);
    if (!fromCoord) {
        um_log_push("[CMD] Ignored - not coordinator");
        return;
    }

    const char *command = payload + 4;
    char ack[220];
    snprintf(ack, sizeof(ack), "command received:%s", command);
    um_mesh.send(senderMac, MESH_TYPE_DATA, pkt->appId,
                 (const uint8_t *)ack, strlen(ack), 4);

    if (strcmp(command, "info") == 0 || strcmp(command, "info:long") == 0) {
        um_sendInfo(senderMac, pkt->appId);
        um_log_push("[CMD] info sent");
    } else if (strcmp(command, "reboot") == 0) {
        um_log_push("[CMD] Rebooting...");
        delay(100);
        ESP.restart();
    } else if (strcmp(command, "update") == 0) {
        um_otaRequested = true;
        um_log_push("[CMD] OTA requested");
    }
}

// -------------------------------------------------------
// Detail overlay
// -------------------------------------------------------
static void um_close_detail(void);

static void um_row_clicked_cb(lv_event_t *e)
{
    if (um_detail_cont) return;
    lv_obj_t *btn = (lv_obj_t *)lv_event_get_target(e);
    int log_idx   = (int)(intptr_t)lv_obj_get_user_data(btn);

    um_detail_cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(um_detail_cont, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_color(um_detail_cont, lv_color_make(8, 8, 8), LV_PART_MAIN);
    lv_obj_set_style_border_color(um_detail_cont, lv_color_make(0, 120, 140), LV_PART_MAIN);
    lv_obj_set_style_border_width(um_detail_cont, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(um_detail_cont, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_all(um_detail_cont, 8, LV_PART_MAIN);
    lv_obj_set_flex_flow(um_detail_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(um_detail_cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(um_detail_cont, 6, LV_PART_MAIN);
    lv_obj_clear_flag(um_detail_cont, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *msg_lbl = lv_label_create(um_detail_cont);
    if (um_mutex && xSemaphoreTake(um_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        lv_label_set_text(msg_lbl, um_log_full[log_idx]);
        xSemaphoreGive(um_mutex);
    } else {
        lv_label_set_text(msg_lbl, "(unavailable)");
    }
    lv_obj_set_style_text_color(msg_lbl, lv_color_make(0, 220, 100), LV_PART_MAIN);
    lv_obj_set_width(msg_lbl, lv_pct(100));
    lv_label_set_long_mode(msg_lbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_flex_grow(msg_lbl, 1);

    lv_obj_t *close_btn = lv_btn_create(um_detail_cont);
    lv_obj_set_width(close_btn, lv_pct(100));
    lv_obj_set_style_bg_color(close_btn, lv_color_make(25, 25, 25), LV_PART_MAIN);
    lv_obj_set_style_bg_color(close_btn, lv_color_make(0, 70, 0),
                              (lv_style_selector_t)(LV_STATE_FOCUSED | LV_PART_MAIN));
    lv_obj_set_style_border_color(close_btn, lv_color_make(60, 60, 60), LV_PART_MAIN);
    lv_obj_set_style_border_width(close_btn, 1, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(close_btn, 0, LV_PART_MAIN);
    lv_obj_add_event_cb(close_btn, [](lv_event_t *ev) { um_close_detail(); }, LV_EVENT_CLICKED, NULL);
    lv_obj_t *close_lbl = lv_label_create(close_btn);
    lv_label_set_text(close_lbl, LV_SYMBOL_LEFT "  Close");
    lv_obj_set_style_text_color(close_lbl, lv_color_make(160, 160, 160), LV_PART_MAIN);
    lv_obj_center(close_lbl);

    lv_group_t *g = lv_group_get_default();
    if (g) {
        lv_group_add_obj(g, close_btn);
        lv_group_focus_obj(close_btn);
        lv_group_focus_freeze(g, true);
    }
}

static void um_close_detail(void)
{
    if (!um_detail_cont) return;
    lv_group_t *g = lv_group_get_default();
    if (g) lv_group_focus_freeze(g, false);
    lv_obj_del(um_detail_cont);
    um_detail_cont = NULL;
    if (um_log_cont && lv_obj_get_child_count(um_log_cont) > 0) {
        lv_obj_t *first = lv_obj_get_child(um_log_cont, 0);
        if (first && g) lv_group_focus_obj(first);
    }
}

// -------------------------------------------------------
// Rebuild message rows
// -------------------------------------------------------
static void um_rebuild_rows(void)
{
    if (!um_log_cont || !um_mutex) return;

    lv_group_t *g = lv_group_get_default();
    lv_obj_clean(um_log_cont);

    if (xSemaphoreTake(um_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;

    int start = (UM_LOG_ROWS + um_logHead - um_logCount) % UM_LOG_ROWS;
    for (int i = um_logCount - 1; i >= 0; i--) {
        int idx = (start + i) % UM_LOG_ROWS;

        lv_obj_t *row = lv_btn_create(um_log_cont);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(row, 0, LV_PART_MAIN);
        lv_obj_set_style_shadow_width(row, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(row, 2, LV_PART_MAIN);
        lv_obj_set_style_pad_ver(row, 1, LV_PART_MAIN);
        lv_obj_set_style_pad_hor(row, 3, LV_PART_MAIN);
        lv_obj_set_style_bg_color(row, lv_color_make(0, 70, 10),
                                  (lv_style_selector_t)(LV_STATE_FOCUSED | LV_PART_MAIN));
        lv_obj_set_style_bg_opa(row, LV_OPA_COVER,
                                (lv_style_selector_t)(LV_STATE_FOCUSED | LV_PART_MAIN));

        lv_obj_t *lbl = lv_label_create(row);
        lv_label_set_text(lbl, um_log[idx]);
        lv_obj_set_width(lbl, lv_pct(100));
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_CLIP);
        lv_obj_set_style_text_color(lbl, lv_color_make(0, 200, 0), LV_PART_MAIN);
        lv_obj_set_style_text_color(lbl, lv_color_make(80, 255, 80),
                                    (lv_style_selector_t)(LV_STATE_FOCUSED | LV_PART_MAIN));

        lv_obj_set_user_data(row, (void *)(intptr_t)idx);
        lv_obj_add_event_cb(row, um_row_clicked_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_add_event_cb(row, [](lv_event_t *ev) {
            lv_obj_scroll_to_view(lv_event_get_target_obj(ev), LV_ANIM_ON);
        }, LV_EVENT_FOCUSED, NULL);

        if (g) lv_group_add_obj(g, row);
    }

    // Focus newest row so rotary can navigate immediately
    if (g && lv_obj_get_child_count(um_log_cont) > 0) {
        lv_group_focus_obj(lv_obj_get_child(um_log_cont, 0));
    }

    xSemaphoreGive(um_mutex);
    um_log_dirty = false;
}

// -------------------------------------------------------
// Discovery task (FreeRTOS)
// -------------------------------------------------------
static void um_discovery_task(void *param)
{
    char line[72];

    WiFi.mode(WIFI_STA);
    esp_err_t wifi_ret = esp_wifi_start();
    if (wifi_ret != ESP_OK) {
        snprintf(line, sizeof(line), "WiFi start failed: %d", wifi_ret);
        um_log_push(line);
        vTaskDelete(NULL);
        return;
    }

    wifi_mode_t mode;
    int wait_count = 0;
    bool wifi_ready = false;
    for (wait_count = 0; wait_count < 40; wait_count++) {
        esp_wifi_get_mode(&mode);
        if (mode == WIFI_MODE_STA) { wifi_ready = true; break; }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    snprintf(line, sizeof(line), "WiFi STA ready after %d checks", wait_count);
    um_log_push(line);
    if (!wifi_ready) {
        um_log_push("WiFi STA not ready!");
        vTaskDelete(NULL);
        return;
    }

    if (!um_mesh.begin(1)) {
        um_log_push("ESP-NOW init FAILED!");
        vTaskDelete(NULL);
        return;
    }
    um_log_push("ESP-NOW init OK");

    um_log_push("Scanning channels 1-13...");
    um_channel = um_mesh.findCoordinatorChannel(NODE_NAME);
    snprintf(line, sizeof(line), "Channel scan returned: %d", um_channel);
    um_log_push(line);

    if (um_channel == 0) {
        um_log_push("No coordinator found");
        um_state = UM_NO_COORD;
        um_task  = NULL;
        vTaskDelete(NULL);
        return;
    }

    snprintf(line, sizeof(line), "Coordinator on ch%d", um_channel);
    um_log_push(line);

    um_mesh.getCoordinatorMac(um_coordMac);
    snprintf(line, sizeof(line), "Coord %02X:%02X:%02X:%02X:%02X:%02X",
             um_coordMac[0], um_coordMac[1], um_coordMac[2],
             um_coordMac[3], um_coordMac[4], um_coordMac[5]);
    um_log_push(line);

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(um_channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    um_mesh.setCoordinatorMac(um_coordMac);
    um_mesh.onReceive(um_on_receive);
    esp_wifi_get_mac(WIFI_IF_STA, um_myMac);

    snprintf(line, sizeof(line), "My MAC %02X:%02X:%02X:%02X:%02X:%02X",
             um_myMac[0], um_myMac[1], um_myMac[2],
             um_myMac[3], um_myMac[4], um_myMac[5]);
    um_log_push(line);

    um_mesh.send(um_coordMac, MESH_TYPE_PING, 0x00,
                 (const uint8_t *)NODE_NAME, strlen(NODE_NAME), 4);
    um_mesh.send(um_coordMac, MESH_TYPE_DATA, 0x06,
                 (const uint8_t *)NODE_NAME, strlen(NODE_NAME), 4);
    um_log_push("Announced. Listening...");

    um_lastHB   = millis() - UM_HB_INTERVAL;
    um_lastTemp = millis() - UM_TEMP_INTERVAL;
    um_state    = UM_CONNECTED;
    um_task     = NULL;
    vTaskDelete(NULL);
}

// -------------------------------------------------------
// 500ms LVGL timer — mesh updates, heartbeat, temp
// -------------------------------------------------------
static void um_timer_cb(lv_timer_t *t)
{
    if (um_state == UM_CONNECTED) {
        um_mesh.update();

        unsigned long now = millis();

        if (now - um_lastHB >= UM_HB_INTERVAL) {
            um_lastHB = now;
            uint8_t hb = 0x01;
            um_mesh.send(um_coordMac, MESH_TYPE_DATA, 0x05, &hb, 1, 4);
            um_mesh.send(um_coordMac, MESH_TYPE_DATA, 0x06,
                         (const uint8_t *)NODE_NAME, strlen(NODE_NAME), 4);
            um_log_push("[HB] Heartbeat sent");
        }

        if (now - um_lastTemp >= UM_TEMP_INTERVAL) {
            um_lastTemp = now;
            float tempC = temperatureRead();
            JsonDocument doc;
            doc["name"] = NODE_NAME;
            doc["temp"] = serialized(String(tempC, 1));
            String payload;
            serializeJson(doc, payload);
            if (um_mesh.sendToCoordinator(0x01, payload)) {
                char line[48];
                snprintf(line, sizeof(line), "[TX] Temp: %.1f C", tempC);
                um_log_push(line);
            } else {
                um_log_push("[TX] Temp send failed");
            }
        }
    }

    if (um_status_lbl) {
        if (um_state == UM_DISCOVERING) {
            static const char *dots[] = {".", "..", "..."};
            char buf[24];
            snprintf(buf, sizeof(buf), "Scanning%s", dots[um_dotPhase % 3]);
            um_dotPhase++;
            lv_label_set_text(um_status_lbl, buf);
            lv_obj_set_style_text_color(um_status_lbl, lv_color_make(255, 160, 0), LV_PART_MAIN);
        } else if (um_state == UM_NO_COORD) {
            lv_label_set_text(um_status_lbl, "No coordinator");
            lv_obj_set_style_text_color(um_status_lbl, lv_color_make(255, 60, 60), LV_PART_MAIN);
        } else {
            lv_label_set_text(um_status_lbl, "Connected");
            lv_obj_set_style_text_color(um_status_lbl, lv_color_make(0, 220, 0), LV_PART_MAIN);
        }
    }

    if (um_info_lbl && um_state == UM_CONNECTED) {
        char buf[56];
        snprintf(buf, sizeof(buf), "Ch:%d  %02X:%02X:%02X:%02X:%02X:%02X",
                 um_channel,
                 um_myMac[0], um_myMac[1], um_myMac[2],
                 um_myMac[3], um_myMac[4], um_myMac[5]);
        lv_label_set_text(um_info_lbl, buf);
        lv_obj_set_style_text_color(um_info_lbl, lv_color_make(150, 150, 150), LV_PART_MAIN);
    }

    if (um_log_dirty && !um_detail_cont) {
        um_rebuild_rows();
    }
}

// -------------------------------------------------------
// Build live mesh screen
// -------------------------------------------------------
static void um_build_mesh_ui(void)
{
    um_root = lv_obj_create(lv_scr_act());
    lv_obj_set_size(um_root, lv_pct(100), lv_pct(100));
    lv_obj_center(um_root);
    lv_obj_set_style_bg_color(um_root, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(um_root, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(um_root, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(um_root, 6, LV_PART_MAIN);
    lv_obj_set_flex_flow(um_root, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(um_root, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(um_root, LV_OBJ_FLAG_SCROLLABLE);

    // Header row
    lv_obj_t *hdr = lv_obj_create(um_root);
    lv_obj_set_width(hdr, lv_pct(100));
    lv_obj_set_height(hdr, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(hdr, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(hdr, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(hdr, 2, LV_PART_MAIN);
    lv_obj_set_flex_flow(hdr, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(hdr, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *hdr_title = lv_label_create(hdr);
    lv_label_set_text(hdr_title, LV_SYMBOL_WIFI " UniversalMesh");
    lv_obj_set_style_text_color(hdr_title, lv_color_make(0, 200, 255), LV_PART_MAIN);

    um_status_lbl = lv_label_create(hdr);
    lv_label_set_text(um_status_lbl, "...");
    lv_obj_set_style_text_color(um_status_lbl, lv_color_make(255, 160, 0), LV_PART_MAIN);

    // Reboot button
    lv_obj_t *exit_btn = lv_btn_create(hdr);
    lv_obj_set_size(exit_btn, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(exit_btn, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_color(exit_btn, lv_color_make(60, 0, 0),
                              (lv_style_selector_t)(LV_STATE_FOCUSED | LV_PART_MAIN));
    lv_obj_set_style_bg_opa(exit_btn, LV_OPA_COVER,
                            (lv_style_selector_t)(LV_STATE_FOCUSED | LV_PART_MAIN));
    lv_obj_set_style_border_width(exit_btn, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(exit_btn, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(exit_btn, 2, LV_PART_MAIN);
    lv_obj_add_event_cb(exit_btn, [](lv_event_t *e) {
        instance.sleep((WakeupSource_t)(WAKEUP_SRC_BOOT_BUTTON | WAKEUP_SRC_ROTARY_BUTTON));
    }, LV_EVENT_CLICKED, NULL);
    lv_obj_t *exit_lbl = lv_label_create(exit_btn);
    lv_label_set_text(exit_lbl, LV_SYMBOL_POWER);
    lv_obj_set_style_text_color(exit_lbl, lv_color_make(200, 60, 60), LV_PART_MAIN);
    lv_obj_center(exit_lbl);

    lv_group_t *g = lv_group_get_default();
    if (g) {
        lv_group_add_obj(g, exit_btn);
        lv_group_focus_obj(exit_btn);
    }

    // Divider
    lv_obj_t *div = lv_obj_create(um_root);
    lv_obj_set_size(div, lv_pct(100), 1);
    lv_obj_set_style_bg_color(div, lv_color_make(50, 50, 50), LV_PART_MAIN);
    lv_obj_set_style_border_width(div, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(div, 0, LV_PART_MAIN);

    // Info line
    um_info_lbl = lv_label_create(um_root);
    lv_label_set_text(um_info_lbl, "Ch:--  --:--:--:--:--:--");
    lv_obj_set_style_text_color(um_info_lbl, lv_color_make(80, 80, 80), LV_PART_MAIN);
    lv_obj_set_width(um_info_lbl, lv_pct(100));

    // Divider
    lv_obj_t *div2 = lv_obj_create(um_root);
    lv_obj_set_size(div2, lv_pct(100), 1);
    lv_obj_set_style_bg_color(div2, lv_color_make(50, 50, 50), LV_PART_MAIN);
    lv_obj_set_style_border_width(div2, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(div2, 0, LV_PART_MAIN);

    // Scrollable message list
    um_log_cont = lv_obj_create(um_root);
    lv_obj_set_width(um_log_cont, lv_pct(100));
    lv_obj_set_flex_grow(um_log_cont, 1);
    lv_obj_set_style_bg_opa(um_log_cont, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(um_log_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(um_log_cont, 0, LV_PART_MAIN);
    lv_obj_set_flex_flow(um_log_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(um_log_cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_add_flag(um_log_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(um_log_cont, LV_DIR_VER);
    lv_obj_set_style_bg_color(um_log_cont, lv_color_make(0, 120, 0), LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_opa(um_log_cont, LV_OPA_COVER, LV_PART_SCROLLBAR);
    lv_obj_set_style_width(um_log_cont, 3, LV_PART_SCROLLBAR);
    lv_obj_set_style_radius(um_log_cont, 2, LV_PART_SCROLLBAR);

    um_mutex = xSemaphoreCreateMutex();
    xTaskCreate(um_discovery_task, "um_disc", 4096, NULL, 5, &um_task);

    um_timer = lv_timer_create(um_timer_cb, 500, NULL);
    lv_timer_ready(um_timer);
}

// -------------------------------------------------------
// Boot logo
// -------------------------------------------------------
static void um_boot_done_cb(lv_timer_t *t)
{
    um_boot_timer = NULL;
    if (um_boot_cont) { lv_obj_del(um_boot_cont); um_boot_cont = NULL; }
    um_build_mesh_ui();
}

static void um_show_boot_screen(void)
{
    um_boot_cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(um_boot_cont, lv_pct(100), lv_pct(100));
    lv_obj_center(um_boot_cont);
    lv_obj_set_style_bg_color(um_boot_cont, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(um_boot_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(um_boot_cont, 0, LV_PART_MAIN);
    lv_obj_set_flex_flow(um_boot_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(um_boot_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(um_boot_cont, 10, LV_PART_MAIN);
    lv_obj_clear_flag(um_boot_cont, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *logo_title = lv_label_create(um_boot_cont);
    lv_label_set_text(logo_title, "UniversalMesh");
    lv_obj_set_style_text_font(logo_title, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_set_style_text_color(logo_title, lv_color_make(0, 210, 255), LV_PART_MAIN);

    lv_obj_t *accent = lv_obj_create(um_boot_cont);
    lv_obj_set_size(accent, 300, 2);
    lv_obj_set_style_bg_color(accent, lv_color_make(0, 90, 140), LV_PART_MAIN);
    lv_obj_set_style_border_width(accent, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(accent, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(accent, 1, LV_PART_MAIN);

    lv_obj_t *sub = lv_label_create(um_boot_cont);
    lv_label_set_text(sub, "ESP-NOW Mesh Network");
    lv_obj_set_style_text_color(sub, lv_color_make(90, 90, 90), LV_PART_MAIN);
    lv_obj_set_style_text_font(sub, &lv_font_montserrat_16, LV_PART_MAIN);

    lv_obj_t *node = lv_label_create(um_boot_cont);
    lv_label_set_text(node, NODE_NAME);
    lv_obj_set_style_text_color(node, lv_color_make(50, 50, 50), LV_PART_MAIN);
    lv_obj_set_style_text_font(node, &lv_font_montserrat_12, LV_PART_MAIN);

    um_boot_timer = lv_timer_create(um_boot_done_cb, 3000, NULL);
    lv_timer_set_repeat_count(um_boot_timer, 1);
}

// -------------------------------------------------------
// Arduino entry points
// -------------------------------------------------------
void setup()
{
    Serial.begin(115200);

    instance.begin();
    beginLvglHelper(instance);

    // Workaround: LV_Helper_v9 creates the default group AFTER
    // registering indevs, so the encoder/keyboard are assigned to NULL.
    // Re-assign all input devices to the actual default group.
    lv_group_t *g = lv_group_get_default();
    if (g) {
        lv_indev_t *indev = NULL;
        while ((indev = lv_indev_get_next(indev)) != NULL) {
            lv_indev_set_group(indev, g);
        }
    }

    instance.setBrightness(DEVICE_MAX_BRIGHTNESS_LEVEL);

    um_show_boot_screen();
}

void loop()
{
    if (um_otaRequested) {
        static bool otaStarted = false;
        if (!otaStarted) {
            otaStarted = true;
            um_log_push("[OTA] Starting update...");
            startOtaUpdate();
        }
        delay(20);
        return;
    }

    lv_timer_handler();
    delay(2);
}
