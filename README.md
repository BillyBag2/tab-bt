# tab-bt

Bluetooth Low Energy analyser for the M5Stack TAB5, using ESP-IDF 6.1,
the official TAB5 BSP, LVGL 9, and NimBLE over ESP-Hosted SDIO.

## Features

- Active ten second scans, names, addresses, address types, RSSI, connectability,
  report counts and raw advertisement/scan-response bytes. Each scan starts a
  fresh device list; up to 48 devices are retained.
- Connect, pair and encrypt with persistent NVS bonds. The touchscreen supports
  passkey entry/display and numeric comparison. Just Works pairing is allowed
  for peers without input/output. Reject cancels pairing; disconnect also cancels
  an outstanding connection attempt. Existing bonds survive repeat-pairing requests.
- Device classification from advertised service UUIDs, GAP appearance and
  discovered GATT services. Unknown/custom devices remain identifiable by address.
- Service and characteristic discovery, readable values, and automatic CCCD
  subscriptions for notifications and indications. The tables hold 32 services
  and 96 characteristics; reaching a table limit is reported.
- Timestamped, bounded packet history with handles, UUIDs, lengths and hex bytes.
  Battery and heart-rate measurements and HID boot keyboard/mouse reports are
  decoded. Other HID reports show raw button/report changes; arbitrary report
  layouts require the peer's HID Report Map and are not guessed.
- Touch controls and Device, GATT, Packets and Audio tabs. Hold packets freezes
  the displayed history while Bluetooth reception continues. The history holds
  roughly 12 KB; individual values display at most 256 bytes. Ordinary GATT reads
  are limited by the negotiated ATT MTU.

This analyses advertisements and GATT traffic received by this application. It
does not capture other devices' connections or provide an over-the-air sniffer.
The GATT and packet tabs describe the active connection, even when another
device is selected in the list. Characteristic property bits are the standard
GATT flags (02 read, 10 notify, 20 indicate).

## Build and run

Use an activated ESP-IDF 6.1 environment. The first build downloads the managed
components; `dependencies.lock` records the resolved versions.

```powershell
idf.py -B build/tab5 -D SDKCONFIG=build/tab5/sdkconfig build
idf.py -B build/tab5 -p COM_PORT flash monitor
```

The separate configuration path ensures `sdkconfig.defaults` enables BLE, TAB5
SDIO pins, PSRAM, the display font and bonding, even if a previous `sdkconfig`
disabled Bluetooth. Replace `COM_PORT` with the TAB5's port. The target defaults
to ESP32-P4 revision v1.0 at 360 MHz, with support for revisions below v3.0 enabled.
This configuration targets the rev1 TAB5; ESP-IDF requires a separate build
configuration for v3.x chips.
Light-sleep clock gating is disabled; the analyser does not use light sleep.
The analyser snapshots and LCD drawing buffers are allocated in PSRAM. LVGL uses
the ESP-IDF heap instead of a fixed 64 KB internal pool, and ESP-Hosted prefers
DMA-capable PSRAM for transport buffers. Larger ordinary allocations prefer PSRAM
so internal RAM remains available for RTOS stacks and driver control structures.
GPIO15 drives the C6 enable/reset line. The project explicitly selects
`ESP_HOSTED_SDIO_RESET_ACTIVE_HIGH`: in this ESP-Hosted driver that means a
high-low-high sequence, leaving CHIP_EN high so the C6 can boot. The opposite
setting leaves the C6 in reset and causes SDIO `send_op_cond` timeouts (0x107).

The onboard ESP32-C6 must run compatible ESP-Hosted coprocessor firmware with
Bluetooth enabled. This project resolves ESP-Hosted 2.13.1; match the coprocessor
to the version in `dependencies.lock`. Factory Wi-Fi firmware may need replacing.
Use the resolved component's `slave` project, select ESP32-C6, its TAB5 board
preset and SDIO transport, and enable Bluetooth controller support. See the
[ESP-Hosted component documentation](https://components.espressif.com/components/espressif/esp_hosted/versions/2.13.1/readme)
and [M5Stack C6 flashing instructions](https://docs.m5stack.com/en/guide/tab5/c6_wifi_restore).
Flashing the P4 application does not update the C6. Radio initialization runs
separately so the interface remains available during startup.

Tap Scan, select a connectable device, then Connect + Pair. Complete any pairing
prompt. Discovery starts after encryption; Inspect retries discovery, including
public attributes if pairing fails. Put peripherals into pairing mode first.
If a peer has lost its bond, clear that bond on both sides before pairing again;
NVS erase also removes this application's other stored settings.

## Audio investigation

The Audio tab detects Published Audio Capabilities (1850), Audio Stream Control
(184E) and Broadcast Audio Scan (184F) services. Their presence indicates advertised
GATT capabilities, and does not prove that audio transport is available.

The stock ESP32-C6 supports BLE rather than Bluetooth Classic, so standard
Classic A2DP/HFP streaming is unavailable with this controller. LE Audio needs
controller/transport ISO support, LC3 encoding/decoding and an audio pipeline.
This firmware inspects audio services but does not implement streaming or codec
playback. Custom vendor audio transported through GATT remains visible as raw
notifications. See Espressif's
[Bluetooth capability overview](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-guides/ble/overview.html).
