#include "analyser.h"
#include "lvgl.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include "esp_err.h"
#include "esp_heap_caps.h"

namespace analyser {
namespace {
Snapshot *view;
lv_obj_t *status_label, *info_label, *attribute_label, *packet_label, *audio_label;
lv_obj_t *device_buttons[max_devices], *device_labels[max_devices];
lv_obj_t *scan_button, *connect_button, *inspect_button;
lv_obj_t *pair_panel, *pair_label, *passkey_field, *keyboard;
lv_obj_t *accept_button, *submit_button;
uint32_t revision = UINT32_MAX;
bool hold_packets;

lv_obj_t *label(lv_obj_t *parent, const char *text)
{
    auto *obj = lv_label_create(parent);
    lv_label_set_text(obj, text);
    lv_obj_set_width(obj, LV_PCT(100));
    lv_label_set_long_mode(obj, LV_LABEL_LONG_WRAP);
    return obj;
}

void send(Action action, uint32_t value = 0)
{
    if (!command(action, value)) lv_label_set_text(status_label, "Command queue full; try again");
}

void clicked(lv_event_t *event)
{
    auto action = static_cast<Action>(reinterpret_cast<uintptr_t>(lv_event_get_user_data(event)));
    send(action);
}

lv_obj_t *button(lv_obj_t *parent, const char *text, Action action)
{
    auto *obj = lv_button_create(parent);
    lv_obj_set_height(obj, 44);
    auto *caption = lv_label_create(obj);
    lv_label_set_text(caption, text);
    lv_obj_center(caption);
    lv_obj_add_event_cb(obj, clicked, LV_EVENT_CLICKED,
                        reinterpret_cast<void *>(static_cast<uintptr_t>(action)));
    return obj;
}

void show(lv_obj_t *obj, bool visible)
{
    if (visible) lv_obj_remove_flag(obj, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
}

void enable(lv_obj_t *obj, bool enabled)
{
    if (enabled) lv_obj_remove_state(obj, LV_STATE_DISABLED);
    else lv_obj_add_state(obj, LV_STATE_DISABLED);
}

void refresh(lv_timer_t *)
{
    snapshot(*view);
    if (revision == view->revision) return;
    revision = view->revision;
    lv_label_set_text(status_label, view->status);
    enable(scan_button, view->ready && !view->connected && !view->busy && !view->scanning);
    enable(connect_button, view->ready && view->selected >= 0 && !view->connected && !view->busy);
    enable(inspect_button, view->connected && !view->busy);
    for (unsigned i = 0; i < max_devices; ++i) {
        show(device_buttons[i], i < view->count);
        if (i >= view->count) continue;
        const auto &d = view->devices[i];
        char text[240];
        snprintf(text, sizeof(text), "%s\n%02X:%02X:%02X:%02X:%02X:%02X   %d dBm\n%s",
                 d.name[0] ? d.name : "Unnamed device", d.address[5], d.address[4], d.address[3],
                 d.address[2], d.address[1], d.address[0], d.rssi,
                 d.type[0] ? d.type : "Unknown BLE device");
        lv_label_set_text(device_labels[i], text);
        if (static_cast<int>(i) == view->selected) lv_obj_add_state(device_buttons[i], LV_STATE_CHECKED);
        else lv_obj_remove_state(device_buttons[i], LV_STATE_CHECKED);
    }
    char info[1600];
    if (view->selected >= 0 && static_cast<unsigned>(view->selected) < view->count) {
        const auto &d = view->devices[view->selected];
        snprintf(info, sizeof(info),
                 "%s\nAddress: %02X:%02X:%02X:%02X:%02X:%02X (%s, type %u)\n"
                 "Type: %s\nSignal: %d dBm   Reports: %lu\nConnectable: %s\n"
                 "Active connection: %s   Encrypted: %s   Bonded: %s\n\nLatest advertisement / scan response:\n%s",
                 d.name[0] ? d.name : "Unnamed", d.address[5], d.address[4], d.address[3],
                 d.address[2], d.address[1], d.address[0], (d.address_type & 1) ? "random" : "public",
                 d.address_type, d.type[0] ? d.type : "Unknown", d.rssi,
                 static_cast<unsigned long>(d.reports), d.connectable ? "yes" : "no",
                 view->connected && view->connected_index == view->selected ? "this device" :
                 view->connected ? "another device" : "no",
                 view->connected_index == view->selected && view->encrypted ? "yes" : "no",
                 view->connected_index == view->selected && view->bonded ? "yes" : "no", d.advertisement);
    } else snprintf(info, sizeof(info), "Tap Scan, then select a device.\nConnect + Pair stores bonds in NVS.\n"
                    "GATT discovery reads public values and subscribes to notifications and indications.\n"
                    "Packet history shows traffic received by this application.");
    lv_label_set_text(info_label, info);
    lv_label_set_text(attribute_label, view->attributes[0] ? view->attributes : "Connect to discover services and characteristics.");
    if (!hold_packets) lv_label_set_text(packet_label, view->packets[0] ? view->packets : "Waiting for advertisements or GATT data...");
    lv_label_set_text(audio_label, view->audio);
    show(pair_panel, view->pairing_action != 0);
    if (view->pairing_action) {
        // NimBLE IO action values: display=3, input=2, numeric comparison=4.
        bool input = view->pairing_action == 2;
        char prompt[160];
        snprintf(prompt, sizeof(prompt), input ? "Enter the six digit passkey shown on the peer" :
                 view->pairing_action == 4 ? "Confirm both devices show %06lu" : "Enter %06lu on the peer",
                 static_cast<unsigned long>(view->passkey));
        lv_label_set_text(pair_label, prompt);
        show(passkey_field, input);
        show(keyboard, input);
        show(submit_button, input);
        show(accept_button, view->pairing_action == 4);
    }
}
} // namespace

void create_ui()
{
    void *memory = heap_caps_malloc(sizeof(Snapshot), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_ERROR_CHECK(memory ? ESP_OK : ESP_ERR_NO_MEM);
    view = new (memory) Snapshot{};
    auto *screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x101827), 0);
    lv_obj_set_style_text_color(screen, lv_color_hex(0xe9eff7), 0);
    lv_obj_set_style_pad_all(screen, 12, 0);
    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);
    auto *title = label(screen, "TAB5  /  Bluetooth analyser");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    auto *toolbar = lv_obj_create(screen);
    lv_obj_set_size(toolbar, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(toolbar, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_style_pad_all(toolbar, 8, 0);
    scan_button = button(toolbar, "Scan", Action::scan);
    button(toolbar, "Stop", Action::stop);
    connect_button = button(toolbar, "Connect + Pair", Action::connect);
    button(toolbar, "Disconnect / Cancel", Action::disconnect);
    inspect_button = button(toolbar, "Inspect", Action::inspect);
    auto *hold = lv_button_create(toolbar);
    auto *hold_text = lv_label_create(hold);
    lv_label_set_text(hold_text, "Hold packets");
    lv_obj_add_flag(hold, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_add_event_cb(hold, [](lv_event_t *e) {
        hold_packets = lv_obj_has_state(static_cast<lv_obj_t *>(lv_event_get_target(e)), LV_STATE_CHECKED);
        revision = UINT32_MAX;
    }, LV_EVENT_VALUE_CHANGED, nullptr);
    status_label = label(screen, "Starting...");
    auto *body = lv_obj_create(screen);
    lv_obj_set_width(body, LV_PCT(100));
    lv_obj_set_flex_grow(body, 1);
    lv_obj_set_flex_flow(body, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(body, 4, 0);
    auto *list = lv_obj_create(body);
    lv_obj_set_size(list, LV_PCT(38), LV_PCT(100));
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(list, 6, 0);
    for (unsigned i = 0; i < max_devices; ++i) {
        auto *obj = lv_button_create(list);
        device_buttons[i] = obj;
        lv_obj_set_size(obj, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
        device_labels[i] = label(obj, "");
        lv_obj_add_event_cb(obj, [](lv_event_t *e) {
            send(Action::select, static_cast<uint32_t>(reinterpret_cast<uintptr_t>(lv_event_get_user_data(e))));
        }, LV_EVENT_CLICKED, reinterpret_cast<void *>(static_cast<uintptr_t>(i)));
    }
    auto *tabs = lv_tabview_create(body);
    lv_obj_set_size(tabs, LV_PCT(60), LV_PCT(100));
    lv_tabview_set_tab_bar_size(tabs, 44);
    info_label = label(lv_tabview_add_tab(tabs, "Device"), "Select a device");
    attribute_label = label(lv_tabview_add_tab(tabs, "GATT"), "");
    packet_label = label(lv_tabview_add_tab(tabs, "Packets"), "");
    audio_label = label(lv_tabview_add_tab(tabs, "Audio"), "");
    pair_panel = lv_obj_create(lv_layer_top());
    lv_obj_set_size(pair_panel, LV_PCT(90), LV_SIZE_CONTENT);
    lv_obj_align(pair_panel, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_flex_flow(pair_panel, LV_FLEX_FLOW_COLUMN);
    pair_label = label(pair_panel, "Pairing");
    passkey_field = lv_textarea_create(pair_panel);
    lv_textarea_set_one_line(passkey_field, true);
    lv_textarea_set_accepted_chars(passkey_field, "0123456789");
    lv_textarea_set_max_length(passkey_field, 6);
    keyboard = lv_keyboard_create(pair_panel);
    lv_keyboard_set_mode(keyboard, LV_KEYBOARD_MODE_NUMBER);
    lv_keyboard_set_textarea(keyboard, passkey_field);
    lv_obj_set_size(keyboard, LV_PCT(100), 170);
    auto *actions = lv_obj_create(pair_panel);
    lv_obj_set_size(actions, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(actions, LV_FLEX_FLOW_ROW_WRAP);
    accept_button = button(actions, "Numbers match", Action::accept);
    submit_button = lv_button_create(actions);
    auto *caption = lv_label_create(submit_button);
    lv_label_set_text(caption, "Submit passkey");
    lv_obj_add_event_cb(submit_button, [](lv_event_t *) {
        const char *value = lv_textarea_get_text(passkey_field);
        if (strlen(value) == 6) send(Action::passkey, strtoul(value, nullptr, 10));
        else lv_label_set_text(pair_label, "Enter exactly six digits");
    }, LV_EVENT_CLICKED, nullptr);
    button(actions, "Reject / Cancel", Action::reject);
    show(pair_panel, false);
    lv_timer_create(refresh, 250, nullptr);
    refresh(nullptr);
}
} // namespace analyser
