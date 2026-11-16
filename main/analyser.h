#pragma once
#include <cstdint>

namespace analyser {
constexpr unsigned max_devices = 48;
struct Device {
    uint8_t address[6]{};
    uint8_t address_type{};
    int8_t rssi{};
    bool connectable{};
    char name[64]{};
    char type[48]{};
    char advertisement[520]{};
    uint32_t reports{};
};
struct Snapshot {
    Device devices[max_devices]{};
    unsigned count{};
    int selected{-1};
    int connected_index{-1};
    bool ready{}, scanning{}, connected{}, busy{}, encrypted{}, bonded{};
    uint8_t pairing_action{};
    uint32_t passkey{};
    uint32_t revision{};
    char status[160]{};
    char attributes[12288]{};
    char packets[12288]{};
    char audio[512]{};
};
enum class Action { scan, stop, select, connect, disconnect, inspect, accept, reject, passkey };
void start();
void snapshot(Snapshot &out);
bool command(Action action, uint32_t value = 0);
void create_ui();
}
