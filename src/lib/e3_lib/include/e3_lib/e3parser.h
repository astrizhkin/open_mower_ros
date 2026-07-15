#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace e3 {

enum CmdType : uint8_t { SET = 0, GET = 1, ACK = 2, NACK = 3 };
enum DataUnit : uint8_t { BYTE = 0, BOOL = 1, INT16 = 2, INT32 = 3, FLOAT32 = 4, INT64 = 5, DOUBLE = 6, CHAR = 7 };

enum CommandKey : uint16_t {
    CMD_ACTIONS_LIST = 0x0200,
    CMD_RESUME       = 0x0210,
    CMD_STOP         = 0x0220,
    CMD_PAUSE        = 0x0230,
    CMD_NEXT         = 0x0240
};

struct E3KVEntry {
    uint16_t key;
    CmdType cmd_type;
    DataUnit data_unit;
    uint16_t payload_length;
    const uint8_t* payload;
};

inline uint32_t crc24q(const uint8_t* data, size_t len) {
    uint32_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint32_t)data[i] << 16);
        for (int j = 0; j < 8; j++) {
            crc <<= 1;
            if (crc & 0x1000000u) crc ^= 0x1864CFBu;
        }
    }
    return crc & 0xFFFFFFu;
}

inline bool validate_e3_crc(const uint8_t* frame, size_t frame_len) {
    if (frame_len < 4) return false;
    size_t data_len = frame_len - 3;
    uint32_t computed = crc24q(frame, data_len);
    uint32_t stored = (uint32_t(frame[data_len]) << 16) |
                      (uint32_t(frame[data_len + 1]) << 8) |
                      uint32_t(frame[data_len + 2]);
    return computed == stored;
}

inline std::vector<E3KVEntry> parse_e3_frame(const uint8_t* frame, size_t frame_len) {
    std::vector<E3KVEntry> entries;
    if (frame_len < 5) return entries;

    if (!validate_e3_crc(frame, frame_len)) return entries;

    uint16_t total_len = (uint16_t(frame[1] & 0x03) << 8) | frame[2];
    size_t data_len = frame_len - 3;
    size_t kv_end = 3 + total_len;
    if (kv_end > data_len) kv_end = data_len;

    size_t pos = 3;
    while (pos + 4 <= kv_end) {
        E3KVEntry entry{};
        entry.key = (uint16_t(frame[pos]) << 8) | frame[pos + 1];
        uint8_t meta = frame[pos + 2];
        entry.cmd_type = static_cast<CmdType>((meta >> 6) & 0x03);
        entry.data_unit = static_cast<DataUnit>((meta >> 3) & 0x07);
        uint16_t plen = (uint16_t(meta & 0x07) << 8) | frame[pos + 3];
        entry.payload_length = plen;
        entry.payload = (pos + 4 < frame_len) ? &frame[pos + 4] : nullptr;
        entries.push_back(entry);
        pos += 4 + plen;
    }
    return entries;
}

inline std::vector<uint8_t> build_e3_frame(const std::vector<E3KVEntry>& entries) {
    std::vector<uint8_t> kv_bytes;
    for (const auto& e : entries) {
        uint16_t plen = e.payload_length;
        kv_bytes.push_back((e.key >> 8) & 0xFF);
        kv_bytes.push_back(e.key & 0xFF);
        uint8_t meta = (uint8_t(e.cmd_type) & 0x03) << 6;
        meta |= (uint8_t(e.data_unit) & 0x07) << 3;
        meta |= (uint8_t(plen >> 8) & 0x07);
        kv_bytes.push_back(meta);
        kv_bytes.push_back(plen & 0xFF);
        if (e.payload && plen > 0) {
            kv_bytes.insert(kv_bytes.end(), e.payload, e.payload + plen);
        }
    }

    std::vector<uint8_t> frame;
    frame.push_back(0xE3);
    uint16_t total_len = static_cast<uint16_t>(kv_bytes.size());
    frame.push_back((total_len >> 8) & 0x03);
    frame.push_back(total_len & 0xFF);
    frame.insert(frame.end(), kv_bytes.begin(), kv_bytes.end());

    uint32_t crc = crc24q(frame.data(), frame.size());
    frame.push_back((crc >> 16) & 0xFF);
    frame.push_back((crc >> 8) & 0xFF);
    frame.push_back(crc & 0xFF);
    return frame;
}

inline std::string cmd_type_name(CmdType ct) {
    switch (ct) {
        case SET:  return "SET";
        case GET:  return "GET";
        case ACK:  return "ACK";
        case NACK: return "NACK";
    }
    return "UNKNOWN";
}

inline std::string key_range_name(uint16_t key) {
    if (key < 0x0100) return "Error";
    if (key < 0x0200) return "State";
    if (key < 0x0300) return "Command";
    if (key < 0x0500) return "Sensor";
    if (key < 0x0900) return "Telemetry";
    return "Unknown";
}

inline size_t data_unit_bytes(DataUnit du) {
    switch (du) {
        case BYTE:    return 1;
        case BOOL:    return 1;
        case INT16:   return 2;
        case INT32:   return 4;
        case FLOAT32: return 4;
        case INT64:   return 8;
        case DOUBLE:  return 8;
        case CHAR:    return 1;
    }
    return 0;
}

} // namespace e3
