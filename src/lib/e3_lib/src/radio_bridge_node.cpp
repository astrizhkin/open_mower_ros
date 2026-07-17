#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <iterator>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>

#include "e3_lib/e3parser.h"
#include "e3_lib/E3KVInput.h"

// ── Helpers ─────────────────────────────────────────────────────────────────

static bool is_immediate_key(uint16_t key) {
    return (key >= 0x0000 && key <= 0x00FF) || (key >= 0x0200 && key <= 0x02FF);
}

// ── Globals ─────────────────────────────────────────────────────────────────

static ros::Publisher          g_radio_write_pub;
static ros::Publisher          g_action_pub;

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
static std::map<uint16_t, std::string>    action_key_to_id;

static double g_batch_interval  = 20.0;
static double g_retry_interval  = 5.0;
static int g_max_retries        = 6;
static int g_batch_max          = 100;
static uint16_t g_sender_id     = 0;

// ── Flush helpers ───────────────────────────────────────────────────────────

static void send_frame(const std::vector<uint8_t>& frame_bytes) {
    std_msgs::UInt8MultiArray msg;
    msg.data = frame_bytes;
    g_radio_write_pub.publish(msg);
}

static void flush_immediate(const e3_lib::E3KVInput& msg) {
    e3::E3KVEntry entry;
    entry.key          = msg.key;
    entry.cmd_type     = static_cast<e3::CmdType>(msg.cmd_type);
    entry.data_unit    = static_cast<e3::DataUnit>(msg.data_unit);
    entry.payload_length = static_cast<uint16_t>(msg.payload.size());
    entry.payload      = msg.payload.data();

    auto frame = e3::build_e3_frame({entry}, g_sender_id);
    send_frame(frame);

    auto now = ros::Time::now();
    auto it = immediate_pending.find(msg.key);
    if (it != immediate_pending.end()) {
        it->second.payload      = msg.payload;
        it->second.last_sent    = now;
        it->second.retransmit_count++;
    } else {
        immediate_pending[msg.key] = {
            msg.key, entry.cmd_type, entry.data_unit,
            msg.payload, now, now, 1
        };
    }
    ROS_INFO("[e3_bridge] Send immediate: key=0x%04X cmd=%s sender=0x%04X frame=%zu bytes",
             msg.key, e3::cmd_type_name(entry.cmd_type).c_str(), g_sender_id, frame.size());
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

    auto frame = e3::build_e3_frame(entries, g_sender_id);
    send_frame(frame);

    ROS_INFO("[e3_bridge] Batch flush: %zu keys, sender=0x%04X frame=%zu bytes", entries.size(), g_sender_id, frame.size());
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

// ── Callbacks ───────────────────────────────────────────────────────────────

static void on_e3_in(const e3_lib::E3KVInput::ConstPtr& msg) {
    uint16_t key = msg->key;

    if (is_immediate_key(key)) {
        flush_immediate(*msg);
    } else {
        auto now = ros::Time::now();
        batch_buffer[key] = {
            msg->key,
            static_cast<e3::CmdType>(msg->cmd_type),
            static_cast<e3::DataUnit>(msg->data_unit),
            msg->payload,
            now,
            ros::Time(),
            0
        };
        ROS_DEBUG("[e3_bridge] Batch queued: key=0x%04X range=%s",
                  key, e3::key_range_name(key).c_str());
    }
}

static void on_e3_radio(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 8) return;

    auto frame = e3::parse_e3_frame_v2(msg->data.data(), msg->data.size());
    if (frame.entries.empty()) return;

    for (const auto& entry : frame.entries) {
        // Handle ACK/NACK
        if (entry.cmd_type == e3::ACK || entry.cmd_type == e3::NACK) {
            auto it = immediate_pending.find(entry.key);
            if (it != immediate_pending.end()) {
                double pending = (ros::Time::now() - it->second.last_sent).toSec();
                ROS_INFO("[e3_bridge] %s received: key=0x%04X sender=0x%04X (pending %.1fs)",
                         e3::cmd_type_name(entry.cmd_type).c_str(), entry.key, frame.sender_id, pending);
                immediate_pending.erase(it);
            } else {
                ROS_WARN("[e3_bridge] %s for unknown key=0x%04X sender=0x%04X",
                         e3::cmd_type_name(entry.cmd_type).c_str(), entry.key, frame.sender_id);
            }
        }
        // Handle incoming commands (ESPHome → ROS, cmd_type SET, key 0x02xx)
        else if (entry.cmd_type == e3::SET && entry.key >= 0x0200 && entry.key <= 0x02FF) {
            auto it = action_key_to_id.find(entry.key);
            if (it != action_key_to_id.end()) {
                std_msgs::String action_msg;
                action_msg.data = it->second;
                g_action_pub.publish(action_msg);
                ROS_INFO("[e3_bridge] Incoming command: key=0x%04X sender=0x%04X -> action_id=%s",
                         entry.key, frame.sender_id, it->second.c_str());
            } else {
                ROS_WARN("[e3_bridge] Incoming command: key=0x%04X sender=0x%04X (no action mapping)", entry.key, frame.sender_id);
            }
        }
        // Log other incoming
        else {
            ROS_INFO("[e3_bridge] Incoming: key=0x%04X sender=0x%04X cmd=%s range=%s plen=%u",
                     entry.key, frame.sender_id, e3::cmd_type_name(entry.cmd_type).c_str(),
                     e3::key_range_name(entry.key).c_str(), entry.payload_length);
        }
    }
}

static void on_batch_timer(const ros::TimerEvent&) {
    flush_batch();
}

static void on_retry_timer(const ros::TimerEvent&) {
    retry_immediate();
}

// ── Action registration callback ────────────────────────────────────────────
//
// Listens for mower_logic publishing its action registry on xbot/available_actions.
// Format: one line per action as "HEX_KEY ACTION_ID"
// Example: "0x0210 mower_logic:idle/resume_mowing\n0x0220 mower_logic:emergency/stop\n"

static void on_available_actions(const std_msgs::String::ConstPtr& msg) {
    action_key_to_id.clear();
    std::istringstream iss(msg->data);
    std::string token;
    while (iss >> token) {
        uint16_t key;
        if (sscanf(token.c_str(), "0x%04hx", &key) != 1) continue;
        std::string action_id;
        if (!(iss >> action_id)) break;
        action_key_to_id[key] = action_id;
        ROS_INFO("[e3_bridge] Registered action: key=0x%04X → %s", key, action_id.c_str());
    }

    ROS_INFO("[e3_bridge] Actions list updated: %zu keys mapped", action_key_to_id.size());

    // Send available keys to ESPHome via E3 actions list frame
    std::vector<uint16_t> keys;
    for (const auto& [k, _] : action_key_to_id) keys.push_back(k);
    if (!keys.empty()) {
        std::vector<uint8_t> payload;
        for (uint16_t k : keys) {
            payload.push_back((k >> 8) & 0xFF);
            payload.push_back(k & 0xFF);
        }
        e3::E3KVEntry entry;
        entry.key = e3::CMD_ACTIONS_LIST;
        entry.cmd_type = e3::SET;
        entry.data_unit = e3::BYTE;
        entry.payload_length = static_cast<uint16_t>(payload.size());
        entry.payload = payload.data();
        auto frame = e3::build_e3_frame({entry}, g_sender_id);
        send_frame(frame);
        ROS_INFO("[e3_bridge] Actions list sent: %zu keys to ESPHome (sender=0x%04X)", keys.size(), g_sender_id);
    }
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
    g_action_pub      = ros::NodeHandle("/").advertise<std_msgs::String>("xbot/action", 1);

    ros::Subscriber e3_in_sub    = pnh.subscribe("e3_in", 10, on_e3_in);
    ros::Subscriber e3_radio_sub = pnh.subscribe("e3_radio", 10, on_e3_radio);
    ros::Subscriber actions_sub  = ros::NodeHandle("/").subscribe("xbot/available_actions", 10, on_available_actions);

    ros::Timer batch_timer = pnh.createTimer(ros::Duration(g_batch_interval), on_batch_timer);
    ros::Timer retry_timer = pnh.createTimer(ros::Duration(g_retry_interval), on_retry_timer);

    // Derive SENDER_ID from ROBOT_ID env var (first 4 hex chars = first 2 bytes of /etc/machine-id)
    {
        const char* robot_id = std::getenv("ROBOT_ID");
        if (robot_id && robot_id[0] && robot_id[1]) {
            uint16_t sid;
            if (sscanf(robot_id, "%04hx", &sid) == 1) {
                g_sender_id = sid;
            } else {
                g_sender_id = 0x0001;
                ROS_WARN("[e3_bridge] Invalid ROBOT_ID '%s', defaulting SENDER_ID=0x0001", robot_id);
            }
        } else {
            g_sender_id = 0x0001;
            ROS_WARN("[e3_bridge] ROBOT_ID not set, defaulting SENDER_ID=0x0001");
        }
    }

    ROS_INFO("[e3_bridge] Started: sender=0x%04X batch=%.0fs retry=%.0fs max_retries=%d",
             g_sender_id, g_batch_interval, g_retry_interval, g_max_retries);

    ros::spin();
    return 0;
}
