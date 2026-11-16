#include "analyser.h"
#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <new>
extern "C" {
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "host/ble_sm.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "store/config/ble_store_config.h"
// NimBLE's ESP-IDF examples declare this initializer explicitly.
void ble_store_config_init(void);
}
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_hosted.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

namespace analyser {
namespace {
Snapshot *state;
SemaphoreHandle_t mutex;
struct StateLock {
    StateLock() { xSemaphoreTake(mutex, portMAX_DELAY); }
    ~StateLock() { xSemaphoreGive(mutex); }
};
struct Command { Action action; uint32_t value; };
QueueHandle_t commands;
ble_npl_callout poll_timer;
uint8_t own_address;
uint16_t connection = BLE_HS_CONN_HANDLE_NONE;
struct Service { uint16_t start, end, uuid; };
struct Characteristic {
    uint16_t definition, value, end, uuid;
    uint8_t properties;
};
Service services[32];
Characteristic characteristics[96];
unsigned service_count, characteristic_count, service_index, characteristic_index;
uint16_t cccd;
bool service_overflow, characteristic_overflow;
bool pacs, ascs, broadcast_audio;

void append(char *dst, size_t capacity, const char *format, ...)
{
    size_t used = strlen(dst);
    if (used >= capacity - 1) return;
    va_list args;
    va_start(args, format);
    vsnprintf(dst + used, capacity - used, format, args);
    va_end(args);
}

void status(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(state->status, sizeof(state->status), format, args);
    va_end(args);
    ++state->revision;
    ESP_LOGI("tab-bt", "%s", state->status);
}

void packet(const char *format, ...)
{
    char line[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    // Keep complete recent lines; memory use does not grow with traffic.
    size_t needed = strlen(line) + 32;
    while (strlen(state->packets) + needed >= sizeof(state->packets)) {
        char *next = strchr(state->packets, '\n');
        if (!next) { state->packets[0] = 0; break; }
        memmove(state->packets, next + 1, strlen(next + 1) + 1);
    }
    append(state->packets, sizeof(state->packets), "%llu ms %s\n",
           static_cast<unsigned long long>(esp_timer_get_time() / 1000), line);
    ++state->revision;
}

uint16_t uuid16(const ble_uuid_t *uuid)
{
    return ble_uuid_u16(uuid);
}

const char *service_type(uint16_t uuid)
{
    switch (uuid) {
    case 0x1812: return "HID / buttons / keyboard / mouse";
    case 0x180d: return "Heart rate sensor";
    case 0x180f: return "Battery service";
    case 0x1809: return "Thermometer";
    case 0x181a: return "Environmental sensor";
    case 0x1816: return "Cycling sensor";
    case 0x184e: return "Audio Stream Control";
    case 0x1850: return "Published Audio Capabilities";
    case 0x184f: return "Broadcast Audio Scan";
    case 0x180a: return "Device information";
    default: return nullptr;
    }
}

void audio_summary()
{
    snprintf(state->audio, sizeof(state->audio),
        "Peer GATT services: PACS %s; ASCS %s; BASS %s.\n"
        "This build inspects audio service values only. Stock ESP32-C6 is BLE-only: "
        "Classic A2DP/HFP is unavailable. LE Audio playback needs an ISO-capable "
        "controller/transport and LC3 pipeline; neither is implemented here.",
        pacs ? "present" : "not found", ascs ? "present" : "not found",
        broadcast_audio ? "present" : "not found");
}

void classify_advertisement(Device &device, const uint8_t *data, size_t length)
{
    // Parse each AD structure with its length checked before reading any field.
    for (size_t pos = 0; pos < length;) {
        const size_t size = data[pos];
        if (!size) break;
        if (size > length - pos - 1) { packet("Malformed advertisement AD length"); break; }
        const uint8_t type = data[pos + 1];
        const uint8_t *value = data + pos + 2;
        const size_t count = size - 1;
        if ((type == 0x08 || type == 0x09) && count) {
            if (type == 0x09 || !device.name[0]) {
                size_t n = std::min(count, sizeof(device.name) - 1);
                for (size_t i = 0; i < n; ++i)
                    device.name[i] = value[i] >= 32 && value[i] < 127 ? value[i] : '.';
                device.name[n] = 0;
            }
        } else if (type == 0x02 || type == 0x03) {
            for (size_t i = 0; i + 1 < count; i += 2) {
                auto uuid = static_cast<uint16_t>(value[i] | value[i + 1] << 8);
                if (const char *kind = service_type(uuid)) {
                    if (uuid != 0x180f || !device.type[0])
                        snprintf(device.type, sizeof(device.type), "%s", kind);
                }
            }
        } else if (type == 0x19 && count >= 2) {
            uint16_t appearance = value[0] | value[1] << 8;
            const char *kind = nullptr;
            if (appearance == 0x03c1) kind = "Keyboard (appearance)";
            else if (appearance == 0x03c2) kind = "Mouse (appearance)";
            else if ((appearance >> 6) == 15) kind = "HID (appearance)";
            if (kind) snprintf(device.type, sizeof(device.type), "%s", kind);
        } else if (type == 0xff && count >= 2 && device.reports == 1) {
            packet("Manufacturer 0x%04x, %u bytes", value[0] | value[1] << 8,
                   static_cast<unsigned>(count - 2));
        }
        pos += size + 1;
    }
}

void scan()
{
    if (!state->ready || state->connected || state->busy) {
        status("Scan requires an idle, synchronized radio"); return;
    }
    if (ble_gap_disc_active()) return;
    ble_gap_disc_params params{};
    params.itvl = 0x80;
    params.window = 0x60;
    params.passive = 0;
    params.filter_duplicates = 0;
    extern int gap_event(ble_gap_event *, void *);
    int rc = ble_gap_disc(own_address, 10000, &params, gap_event, nullptr);
    state->scanning = rc == 0;
    if (!rc) {
        state->count = 0;
        state->selected = -1;
        for (auto &device : state->devices) device = Device{};
    }
    status(rc ? "Scan failed (%d)" : "Scanning for 10 seconds", rc);
}

void dump_value(uint16_t handle, os_mbuf *buffer, const char *source)
{
    uint8_t bytes[256];
    size_t full_length = OS_MBUF_PKTLEN(buffer);
    uint16_t length = std::min(full_length, sizeof(bytes));
    if (os_mbuf_copydata(buffer, 0, length, bytes)) return;
    char hex[sizeof(bytes) * 3 + 1]{};
    for (unsigned i = 0; i < length; ++i)
        snprintf(hex + i * 3, 4, "%02X ", bytes[i]);
    uint16_t uuid = 0;
    for (unsigned i = 0; i < characteristic_count; ++i)
        if (characteristics[i].value == handle) uuid = characteristics[i].uuid;
    packet("%s handle=0x%04x uuid=0x%04x len=%u%s: %s", source, handle, uuid,
           static_cast<unsigned>(full_length), full_length > length ? " (truncated)" : "", hex);
    if (uuid == 0x2a19 && length)
        packet("Battery: %u%%", bytes[0]);
    else if (uuid == 0x2a37 && length >= 2) {
        if (!(bytes[0] & 1)) packet("Heart rate: %u bpm", bytes[1]);
        else if (length >= 3) packet("Heart rate: %u bpm", bytes[1] | bytes[2] << 8);
    } else if (uuid == 0x2a22 && length >= 8) {
        packet("Boot keyboard: modifiers=0x%02x keys=%02x %02x %02x %02x %02x %02x",
               bytes[0], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7]);
    } else if (uuid == 0x2a33 && length >= 3) {
        packet("Boot mouse: left=%u right=%u middle=%u dx=%d dy=%d",
               !!(bytes[0] & 1), !!(bytes[0] & 2), !!(bytes[0] & 4),
               static_cast<int8_t>(bytes[1]), static_cast<int8_t>(bytes[2]));
    } else if (uuid == 0x2a4d) {
        packet("HID report/button state updated; layout is defined by the peer Report Map (2A4B)");
    }
}

void next_characteristic();
int subscribed(uint16_t conn, const ble_gatt_error *error, ble_gatt_attr *, void *)
{
    StateLock lock;
    if (conn != connection) return 0;
    packet("CCCD 0x%04x: %s (%d)", cccd, error->status ? "failed" : "subscribed", error->status);
    ++characteristic_index;
    next_characteristic();
    return 0;
}

int descriptor_found(uint16_t conn, const ble_gatt_error *error, uint16_t,
                     const ble_gatt_dsc *descriptor, void *)
{
    StateLock lock;
    if (conn != connection) return 0;
    if (!error->status) {
        if (uuid16(&descriptor->uuid.u) == 0x2902) cccd = descriptor->handle;
        return 0;
    }
    if (error->status != BLE_HS_EDONE)
        packet("Descriptor discovery failed (%d)", error->status);
    if (cccd) {
        uint8_t value[2] = {
            static_cast<uint8_t>((characteristics[characteristic_index].properties & BLE_GATT_CHR_F_NOTIFY) ? 1 : 2), 0};
        int rc = ble_gattc_write_flat(connection, cccd, value, sizeof(value), subscribed, nullptr);
        if (!rc) return 0;
        packet("Subscription start failed (%d)", rc);
    }
    ++characteristic_index;
    next_characteristic();
    return 0;
}

void discover_descriptors()
{
    auto &chr = characteristics[characteristic_index];
    if ((chr.properties & (BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE)) && chr.end > chr.value) {
        cccd = 0;
        int rc = ble_gattc_disc_all_dscs(connection, chr.value, chr.end, descriptor_found, nullptr);
        if (!rc) return;
        packet("Descriptor request failed (%d)", rc);
    }
    ++characteristic_index;
    next_characteristic();
}

int value_read(uint16_t conn, const ble_gatt_error *error, ble_gatt_attr *attr, void *)
{
    StateLock lock;
    if (conn != connection) return 0;
    if (!error->status && attr && attr->om) dump_value(attr->handle, attr->om, "READ");
    else packet("Read failed (%d)", error->status);
    discover_descriptors();
    return 0;
}

void next_characteristic()
{
    // Immediate errors advance iteratively rather than recursing through every attribute.
    while (characteristic_index < characteristic_count) {
        const auto &chr = characteristics[characteristic_index];
        if (chr.properties & BLE_GATT_CHR_F_READ) {
            int rc = ble_gattc_read(connection, chr.value, value_read, nullptr);
            if (!rc) return;
            packet("Read request failed (%d)", rc);
        }
        if ((chr.properties & (BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE)) && chr.end > chr.value) {
            cccd = 0;
            int rc = ble_gattc_disc_all_dscs(connection, chr.value, chr.end, descriptor_found, nullptr);
            if (!rc) return;
            packet("Descriptor request failed (%d)", rc);
        }
        ++characteristic_index;
    }
    state->busy = false;
    audio_summary();
    status("Ready: %u services, %u characteristics%s", service_count, characteristic_count,
           (service_overflow || characteristic_overflow) ? " (table limit reached)" : "");
}

void discover_characteristics();
int characteristic_found(uint16_t conn, const ble_gatt_error *error, const ble_gatt_chr *chr, void *)
{
    StateLock lock;
    if (conn != connection) return 0;
    if (!error->status) {
        // Close the preceding characteristic's descriptor range at this declaration.
        if (characteristic_count && characteristics[characteristic_count - 1].end == services[service_index].end)
            characteristics[characteristic_count - 1].end = chr->def_handle - 1;
        if (characteristic_count == 96) { characteristic_overflow = true; return 0; }
        characteristics[characteristic_count++] = {
            chr->def_handle, chr->val_handle, services[service_index].end,
            uuid16(&chr->uuid.u), chr->properties};
        char uuid[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&chr->uuid.u, uuid);
        append(state->attributes, sizeof(state->attributes), "  %s value=%04x properties=%02x\n",
               uuid, chr->val_handle, chr->properties);
        ++state->revision;
        return 0;
    }
    if (error->status != BLE_HS_EDONE) packet("Characteristic discovery failed (%d)", error->status);
    ++service_index;
    discover_characteristics();
    return 0;
}

void discover_characteristics()
{
    while (service_index < service_count) {
        auto &svc = services[service_index];
        int rc = ble_gattc_disc_all_chrs(connection, svc.start, svc.end, characteristic_found, nullptr);
        if (!rc) return;
        packet("Characteristic request failed (%d)", rc);
        ++service_index;
    }
    characteristic_index = 0;
    next_characteristic();
}

int service_found(uint16_t conn, const ble_gatt_error *error, const ble_gatt_svc *svc, void *)
{
    StateLock lock;
    if (conn != connection) return 0;
    if (!error->status) {
        if (service_count == 32) { service_overflow = true; return 0; }
        uint16_t uuid = uuid16(&svc->uuid.u);
        services[service_count++] = {svc->start_handle, svc->end_handle, uuid};
        pacs |= uuid == 0x1850;
        ascs |= uuid == 0x184e;
        broadcast_audio |= uuid == 0x184f;
        char text[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&svc->uuid.u, text);
        const char *kind = service_type(uuid);
        append(state->attributes, sizeof(state->attributes), "%s [%04x-%04x] %s\n",
               text, svc->start_handle, svc->end_handle, kind ? kind : "Custom/other service");
        if (kind && state->connected_index >= 0 && uuid != 0x180a && uuid != 0x180f)
            snprintf(state->devices[state->connected_index].type, sizeof(Device::type), "%s", kind);
        ++state->revision;
        return 0;
    }
    if (error->status != BLE_HS_EDONE) packet("Service discovery failed (%d)", error->status);
    service_index = 0;
    discover_characteristics();
    return 0;
}

void inspect()
{
    if (!state->connected || state->busy) { status("Inspection requires an idle connection"); return; }
    service_count = characteristic_count = 0;
    service_overflow = characteristic_overflow = false;
    pacs = ascs = broadcast_audio = false;
    state->attributes[0] = 0;
    snprintf(state->audio, sizeof(state->audio), "Discovering peer audio services...");
    int rc = ble_gattc_disc_all_svcs(connection, service_found, nullptr);
    state->busy = rc == 0;
    status(rc ? "Service request failed (%d)" : "Discovering GATT services...", rc);
}

int gap_event(ble_gap_event *event, void *)
{
    // All callbacks and commands run on the NimBLE host task. The UI only copies state->
    xSemaphoreTake(mutex, portMAX_DELAY);
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        const auto &report = event->disc;
        unsigned index = 0;
        for (; index < state->count; ++index)
            if (state->devices[index].address_type == report.addr.type &&
                !memcmp(state->devices[index].address, report.addr.val, 6)) break;
        if (index == state->count) {
            if (state->count == max_devices) break;
            ++state->count;
            memcpy(state->devices[index].address, report.addr.val, 6);
            state->devices[index].address_type = report.addr.type;
        }
        auto &device = state->devices[index];
        device.rssi = report.rssi;
        ++device.reports;
        if (report.event_type != BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP)
            device.connectable = report.event_type == BLE_HCI_ADV_RPT_EVTYPE_ADV_IND ||
                                 report.event_type == BLE_HCI_ADV_RPT_EVTYPE_DIR_IND;
        device.advertisement[0] = 0;
        for (unsigned i = 0; i < report.length_data; ++i)
            append(device.advertisement, sizeof(device.advertisement), "%02X ", report.data[i]);
        classify_advertisement(device, report.data, report.length_data);
        ++state->revision;
        break;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        state->scanning = false;
        status("Scan complete (%u devices, reason %d)%s", state->count, event->disc_complete.reason,
               state->count == max_devices ? "; device table full" : "");
        break;
    case BLE_GAP_EVENT_CONNECT:
        state->busy = false;
        if (event->connect.status) {
            status("Connection failed (%d)", event->connect.status);
            state->connected_index = -1;
            break;
        }
        connection = event->connect.conn_handle;
        ble_gattc_exchange_mtu(connection, nullptr, nullptr);
        state->connected = true;
        state->busy = true;
        status("Connected; requesting encryption / pairing");
        {
            int rc = ble_gap_security_initiate(connection);
            if (rc) {
                state->busy = false;
                status("Security request failed (%d); Inspect can try public attributes", rc);
            }
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        connection = BLE_HS_CONN_HANDLE_NONE;
        state->connected_index = -1;
        state->connected = state->encrypted = state->bonded = state->busy = false;
        state->pairing_action = 0;
        characteristic_count = service_count = 0;
        status("Disconnected (reason %d)", event->disconnect.reason);
        break;
    case BLE_GAP_EVENT_ENC_CHANGE: {
        state->busy = false;
        state->pairing_action = 0;
        ble_gap_conn_desc desc{};
        if (!event->enc_change.status && !ble_gap_conn_find(connection, &desc)) {
            state->encrypted = desc.sec_state.encrypted;
            state->bonded = desc.sec_state.bonded;
            packet("Encryption=%d bonded=%d authenticated=%d", state->encrypted, state->bonded,
                   desc.sec_state.authenticated);
            inspect();
        } else status("Pairing/encryption failed (%d); retry connection or Inspect", event->enc_change.status);
        break;
    }
    case BLE_GAP_EVENT_PASSKEY_ACTION: {
        state->pairing_action = event->passkey.params.action;
        state->passkey = event->passkey.params.numcmp;
        if (state->pairing_action == BLE_SM_IOACT_DISP) {
            ble_sm_io io{};
            io.action = BLE_SM_IOACT_DISP;
            io.passkey = esp_random() % 1000000;
            state->passkey = io.passkey;
            int rc = ble_sm_inject_io(connection, &io);
            status("Enter %06lu on peer (status %d)", static_cast<unsigned long>(state->passkey), rc);
        } else status("Pairing needs user input or numeric confirmation");
        break;
    }
    case BLE_GAP_EVENT_NOTIFY_RX:
        if (event->notify_rx.conn_handle == connection)
            dump_value(event->notify_rx.attr_handle, event->notify_rx.om,
                       event->notify_rx.indication ? "INDICATE" : "NOTIFY");
        break;
    case BLE_GAP_EVENT_REPEAT_PAIRING:
        status("Peer requested repeat pairing; existing bond preserved");
        xSemaphoreGive(mutex);
        return BLE_GAP_REPEAT_PAIRING_IGNORE;
    default: break;
    }
    xSemaphoreGive(mutex);
    return 0;
}

void process_command(const Command &cmd)
{
    switch (cmd.action) {
    case Action::scan: scan(); break;
    case Action::stop:
        if (ble_gap_disc_active()) {
            int rc = ble_gap_disc_cancel();
            state->scanning = false;
            status("Scan stopped (%d)", rc);
        }
        break;
    case Action::select:
        if (cmd.value < state->count) { state->selected = cmd.value; ++state->revision; }
        break;
    case Action::connect: {
        if (!state->ready || state->connected || state->busy || state->selected < 0) break;
        const auto &device = state->devices[state->selected];
        if (!device.connectable) { status("Selected advertisement is not connectable"); break; }
        if (ble_gap_disc_active()) {
            int rc = ble_gap_disc_cancel();
            if (rc) { status("Cannot stop scan (%d)", rc); break; }
            state->scanning = false;
        }
        ble_addr_t address{};
        address.type = device.address_type;
        memcpy(address.val, device.address, 6);
        int rc = ble_gap_connect(own_address, &address, 10000, nullptr, gap_event, nullptr);
        if (!rc) state->connected_index = state->selected;
        state->busy = rc == 0;
        status(rc ? "Connect failed (%d)" : "Connecting...", rc);
        break;
    }
    case Action::disconnect:
        if (state->connected) {
            int rc = ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
            status("Disconnect requested (%d)", rc);
        } else if (state->busy) {
            int rc = ble_gap_conn_cancel();
            status("Connection cancellation requested (%d)", rc);
        }
        break;
    case Action::inspect: inspect(); break;
    case Action::accept:
    case Action::reject:
    case Action::passkey: {
        if (!state->connected || !state->pairing_action) break;
        if (cmd.action == Action::reject) {
            ble_gap_terminate(connection, BLE_ERR_AUTH_FAIL);
            state->pairing_action = 0;
            break;
        }
        ble_sm_io io{};
        io.action = state->pairing_action;
        if (io.action == BLE_SM_IOACT_NUMCMP && cmd.action == Action::accept) io.numcmp_accept = 1;
        else if (io.action == BLE_SM_IOACT_INPUT && cmd.action == Action::passkey && cmd.value <= 999999)
            io.passkey = cmd.value;
        else { status("Use the requested pairing action"); break; }
        int rc = ble_sm_inject_io(connection, &io);
        if (!rc) state->pairing_action = 0;
        status("Pairing response (%d)", rc);
        break;
    }
    }
}

void poll(ble_npl_event *)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    Command cmd;
    while (xQueueReceive(commands, &cmd, 0) == pdTRUE) process_command(cmd);
    xSemaphoreGive(mutex);
    ble_npl_callout_reset(&poll_timer, ble_npl_time_ms_to_ticks32(100));
}

void synced()
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    int rc = ble_hs_util_ensure_addr(0);
    if (!rc) rc = ble_hs_id_infer_auto(0, &own_address);
    state->ready = rc == 0;
    status(rc ? "BLE address initialization failed (%d)" : "Bluetooth ready", rc);
    xSemaphoreGive(mutex);
    ble_npl_callout_reset(&poll_timer, ble_npl_time_ms_to_ticks32(100));
}

void reset(int reason)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    state->ready = state->scanning = state->connected = state->busy = state->encrypted = state->bonded = false;
    state->pairing_action = 0;
    connection = BLE_HS_CONN_HANDLE_NONE;
    state->connected_index = -1;
    service_count = characteristic_count = 0;
    status("Bluetooth host reset (%d); waiting for synchronization", reason);
    xSemaphoreGive(mutex);
}

void host_task(void *)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void initialize_radio(void *)
{
    int rc = esp_hosted_connect_to_slave();
    if (!rc) rc = esp_hosted_bt_controller_init();
    if (!rc) rc = esp_hosted_bt_controller_enable();
    if (!rc) rc = nimble_port_init();
    if (rc) {
        {
            StateLock lock;
            status("Radio startup failed (%d); check matching C6 BLE firmware", rc);
        }
        vTaskDelete(nullptr);
        return;
    }
    ble_hs_cfg.sync_cb = synced;
    ble_hs_cfg.reset_cb = reset;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_KEYBOARD_DISPLAY;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_store_config_init();
    ble_npl_callout_init(&poll_timer, nimble_port_get_dflt_eventq(), poll, nullptr);
    nimble_port_freertos_init(host_task);
    vTaskDelete(nullptr);
}
} // namespace

void start()
{
    // These large histories do not need cache-disabled access like RTOS stacks do.
    void *memory = heap_caps_malloc(sizeof(Snapshot), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_ERROR_CHECK(memory ? ESP_OK : ESP_ERR_NO_MEM);
    state = new (memory) Snapshot{};
    mutex = xSemaphoreCreateMutex();
    commands = xQueueCreate(16, sizeof(Command));
    configASSERT(mutex && commands);
    status("Starting Bluetooth; C6 must run matching ESP-Hosted BLE firmware");
    audio_summary();
    configASSERT(xTaskCreate(initialize_radio, "radio-init", 8192, nullptr, 5, nullptr) == pdPASS);
}

void snapshot(Snapshot &out)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    out = *state;
    xSemaphoreGive(mutex);
}

bool command(Action action, uint32_t value)
{
    Command cmd{action, value};
    return xQueueSend(commands, &cmd, 0) == pdTRUE;
}
} // namespace analyser
