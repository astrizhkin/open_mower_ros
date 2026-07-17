#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>

#include "e3_lib/e3parser.h"
#include "e3_lib/E3KVInput.h"
#include "e3_lib/ScheduleE3KV.h"

// ── Helpers ─────────────────────────────────────────────────────────────────

static bool is_immediate_key(uint16_t key) {
    return (key >= 0x0000 && key <= 0x00FF) || (key >= 0x0200 && key <= 0x02FF);
}

// ── Globals ─────────────────────────────────────────────────────────────────

static ros::Publisher          g_radio_write_pub;
static ros::Publisher          g_rx_pub;

struct BufferedEntry {
    uint16_t key;
    e3::CmdType cmd_type;
    e3::DataUnit data_unit;
    std::vector<uint8_t> payload;
    ros::Time last_updated;
    ros::Time last_sent;
    uint8_t retransmit_count;
};

static std::map<uint16_t, BufferedEntry>  batch_buffer;
static std::map<uint16_t, BufferedEntry>  immediate_pending;

static double g_batch_interval  = 20.0;
static double g_retry_interval  = 5.0;
static int g_max_retries        = 6;
static int g_batch_max          = 100;

// ── Flush helpers ───────────────────────────────────────────────────────────

static void send_payload(const std::vector<uint8_t>& payload_bytes) {
    std_msgs::UInt8MultiArray msg;
    msg.data = payload_bytes;
    g_radio_write_pub.publish(msg);
}

static void flush_immediate(const e3_lib::E3KVInput& kv) {
    e3::E3KVEntry entry;
    entry.key            = kv.key;
    entry.cmd_type       = static_cast<e3::CmdType>(kv.cmd_type);
    entry.data_unit      = static_cast<e3::DataUnit>(kv.data_unit);
    entry.payload_length = static_cast<uint16_t>(kv.payload.size());
    entry.payload        = kv.payload.data();

    auto payload = e3::build_e3_payload({entry});
    send_payload(payload);

    auto now = ros::Time::now();
    auto it = immediate_pending.find(kv.key);
    if (it != immediate_pending.end()) {
        it->second.payload           = kv.payload;
        it->second.last_sent         = now;
        it->second.retransmit_count++;
    } else {
        immediate_pending[kv.key] = {
            kv.key, entry.cmd_type, entry.data_unit,
            kv.payload, now, now, 1
        };
    }
    ROS_INFO("[e3_bridge] Send immediate: key=0x%04X cmd=%s payload=%zu bytes",
             kv.key, e3::cmd_type_name(entry.cmd_type).c_str(), payload.size());
}

static void queue_batch(const e3_lib::E3KVInput& kv) {
    auto now = ros::Time::now();
    batch_buffer[kv.key] = {
        kv.key,
        static_cast<e3::CmdType>(kv.cmd_type),
        static_cast<e3::DataUnit>(kv.data_unit),
        kv.payload,
        now,
        ros::Time(),
        0
    };
    ROS_DEBUG("[e3_bridge] Batch queued: key=0x%04X range=%s",
              kv.key, e3::key_range_name(kv.key).c_str());
}

static void flush_batch() {
    if (batch_buffer.empty()) return;

    if (static_cast<int>(batch_buffer.size()) > g_batch_max) {
        ROS_WARN("[e3_bridge] batch buffer has %zu entries (threshold: %d), dropping oldest",
                 batch_buffer.size(), g_batch_max);
        int to_drop = static_cast<int>(batch_buffer.size()) - g_batch_max;
        auto drop_it = batch_buffer.begin();
        std::advance(drop_it, to_drop);
        batch_buffer.erase(batch_buffer.begin(), drop_it);
    }

    std::vector<e3::E3KVEntry> entries;
    entries.reserve(batch_buffer.size());
    for (const auto& [key, entry] : batch_buffer) {
        e3::E3KVEntry e;
        e.key            = entry.key;
        e.cmd_type       = entry.cmd_type;
        e.data_unit      = entry.data_unit;
        e.payload_length = static_cast<uint16_t>(entry.payload.size());
        e.payload        = entry.payload.data();
        entries.push_back(e);
    }

    auto payload = e3::build_e3_payload(entries);
    send_payload(payload);

    ROS_INFO("[e3_bridge] Batch flush: %zu keys, payload=%zu bytes", entries.size(), payload.size());
    batch_buffer.clear();
}

static void retry_immediate() {
    auto now = ros::Time::now();
    auto it = immediate_pending.begin();
    while (it != immediate_pending.end()) {
        double elapsed = (now - it->second.last_sent).toSec();
        if (elapsed >= g_retry_interval && it->second.retransmit_count < g_max_retries) {
            e3_lib::E3KVInput remsg;
            remsg.key            = it->second.key;
            remsg.cmd_type       = static_cast<uint8_t>(it->second.cmd_type);
            remsg.data_unit      = static_cast<uint8_t>(it->second.data_unit);
            remsg.payload        = it->second.payload;
            flush_immediate(remsg);
            ROS_WARN("[e3_bridge] Retry #%u for key=0x%04X (no ACK after %.1fs)",
                     it->second.retransmit_count, it->second.key, elapsed);
            it++;
        } else if (it->second.retransmit_count >= g_max_retries) {
            ROS_ERROR("[e3_bridge] ERROR: max retries (%u) exceeded for key=0x%04X",
                      g_max_retries, it->second.key);
            it = immediate_pending.erase(it);
        } else {
            it++;
        }
    }
}

// ── Service handler ─────────────────────────────────────────────────────────

static bool schedule_e3kv(e3_lib::ScheduleE3KV::Request& req,
                           e3_lib::ScheduleE3KV::Response& res) {
    if (req.kvs.empty()) {
        res.success = false;
        res.message = "empty request";
        return true;
    }

    size_t immediate = 0, batched = 0;
    for (const auto& kv : req.kvs) {
        if (is_immediate_key(kv.key)) {
            flush_immediate(kv);
            immediate++;
        } else {
            queue_batch(kv);
            batched++;
        }
    }
    res.success = true;
    res.message = std::to_string(immediate) + " immediate, " + std::to_string(batched) + " batched";
    return true;
}

// ── Callbacks ───────────────────────────────────────────────────────────────

static void on_e3_radio(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
    if (msg->data.empty()) return;

    auto entries = e3::parse_e3_payload(msg->data.data(), msg->data.size());
    if (entries.empty()) return;

    for (const auto& entry : entries) {
        // Handle ACK/NACK internally
        if (entry.cmd_type == e3::ACK || entry.cmd_type == e3::NACK) {
            auto it = immediate_pending.find(entry.key);
            if (it != immediate_pending.end()) {
                double pending = (ros::Time::now() - it->second.last_sent).toSec();
                ROS_INFO("[e3_bridge] %s received: key=0x%04X (pending %.1fs)",
                         e3::cmd_type_name(entry.cmd_type).c_str(), entry.key, pending);
                immediate_pending.erase(it);
            } else {
                ROS_WARN("[e3_bridge] %s for unknown key=0x%04X",
                         e3::cmd_type_name(entry.cmd_type).c_str(), entry.key);
            }
        } else {
            // Forward all non-ACK/NACK entries to rx_e3kv
            e3_lib::E3KVInput kv_msg;
            kv_msg.key = entry.key;
            kv_msg.cmd_type = static_cast<uint8_t>(entry.cmd_type);
            kv_msg.data_unit = static_cast<uint8_t>(entry.data_unit);
            if (entry.payload && entry.payload_length > 0) {
                kv_msg.payload.assign(entry.payload, entry.payload + entry.payload_length);
            }
            g_rx_pub.publish(kv_msg);
        }
    }
}

static void on_batch_timer(const ros::TimerEvent&) {
    flush_batch();
}

static void on_retry_timer(const ros::TimerEvent&) {
    retry_immediate();
}

// ── main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    ros::init(argc, argv, "radio_bridge_node");
    ros::NodeHandle pnh("~");

    pnh.param("batch_interval", g_batch_interval, 20.0);
    pnh.param("retry_interval", g_retry_interval, 5.0);
    pnh.param("max_retries", g_max_retries, 6);
    pnh.param("batch_max_entries", g_batch_max, 100);

    g_radio_write_pub = pnh.advertise<std_msgs::UInt8MultiArray>("radio_write", 10);
    g_rx_pub          = pnh.advertise<e3_lib::E3KVInput>("rx_e3kv", 10);

    ros::ServiceServer schedule_srv = pnh.advertiseService("schedule_e3kv", schedule_e3kv);
    ros::Subscriber radio_sub       = pnh.subscribe("e3_radio", 10, on_e3_radio);

    ros::Timer batch_timer = pnh.createTimer(ros::Duration(g_batch_interval), on_batch_timer);
    ros::Timer retry_timer = pnh.createTimer(ros::Duration(g_retry_interval), on_retry_timer);

    ROS_INFO("[e3_bridge] Started: batch=%.0fs retry=%.0fs max_retries=%d",
             g_batch_interval, g_retry_interval, g_max_retries);

    ros::spin();
    return 0;
}
