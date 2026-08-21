// uss_slowdown: graded ultrasonic slowdown for the autonomous drive.
//
// Subscribes to the raw US ranges (/mower/uss), the autonomous velocity
// command (/nav_vel) and the high-level state (mower_logic/current_state).
// When the robot is autonomous it overrides the velocity through twist_mux
// (/uss_vel, priority 70) so the wheels taper from full speed down to a stop
// as an obstacle closes in, and auto-resume when the path clears.
//
// Config: sensor_behavior + uss_input_config come from the mower_logic
// dynamic-reconfigure config (live). The 6 US tuning params are node-local.
//
// Modeled on antislip.cpp (same package).

#include "ros/ros.h"
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <mower_msgs/HighLevelStatus.h>
#include <dynamic_reconfigure/client.h>
#include "mower_logic/MowerLogicConfig.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

// ---- constants ----
static const int    USS_COUNT             = 5;
static const double USS_STALE_TIMEOUT     = 1.0;   // s; a reading older than this is ignored
static const double USS_MAX_RANGE         = 2.55;  // m; >= this means "no obstacle"
static const double USS_FILT_SCALE        = 65536.0; // Q16 scale: alpha = uss_average_coef / USS_FILT_SCALE
static const double USS_SIDE_CLOSING_FACTOR = 0.6; // side sensors close at ~0.6x forward speed
static const double NAV_VEL_TIMEOUT       = 2.0;   // s; /nav_vel older than this is "stale" (nav publishes at ~1 Hz)

// ---- node-local tuning params (private) ----
static double uss_front_slowdown_percent;
static double uss_side_slowdown_percent;
static double uss_front_stop_distance;
static double uss_side_stop_distance;
static double uss_average_threshold;
static int    uss_average_coef;

// ---- state ----
struct UssReading {
  double    range;
  ros::Time stamp;
};
static UssReading uss_readings[USS_COUNT];
static bool uss_enabled[USS_COUNT] = {false};

static geometry_msgs::Twist last_nav_vel;
static ros::Time last_nav_vel_time;

static uint8_t hl_state = mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_NULL;

static double uss_average = 1.0;  // low-pass filtered average (m), seeded at 1.0 m

static mower_logic::MowerLogicConfig cfg;
static dynamic_reconfigure::Client<mower_logic::MowerLogicConfig> *reconfigClient;

static ros::Publisher uss_vel_pub;

// ---- helpers ----
static bool isSideSensor(int i) {
  return i == 0 || i == 4;
}

// Predict the current range of sensor i: the stored range minus how far the
// bot has moved toward the object since the measurement was taken.
static double predictedRange(int i, const ros::Time &now) {
  double age = (now - uss_readings[i].stamp).toSec();
  if (age < 0.0) age = 0.0;
  double closing = (isSideSensor(i) ? USS_SIDE_CLOSING_FACTOR : 1.0) * last_nav_vel.linear.x;
  double predicted = uss_readings[i].range - closing * age;
  if (predicted < 0.0) predicted = 0.0;
  return predicted;
}

// Per-sensor slowdown factor in [0,1]: 1 = full speed, 0 = stop.
static double sensorFactor(int i, const ros::Time &now) {
  if (!uss_enabled[i]) return 1.0;

  // validity: fresh, finite, and within range
  double age = (now - uss_readings[i].stamp).toSec();
  if (age < 0.0 || age > USS_STALE_TIMEOUT) return 1.0;
  double range = uss_readings[i].range;
  if (!std::isfinite(range) || range >= USS_MAX_RANGE) return 1.0;

  double dist = predictedRange(i, now);

  double stop_distance = isSideSensor(i) ? uss_side_stop_distance : uss_front_stop_distance;
  double percent = isSideSensor(i) ? uss_side_slowdown_percent : uss_front_slowdown_percent;
  double slowdown_distance = std::max(percent * uss_average, stop_distance);  // adaptive slowdown start, clamped >= T

  if (dist <= stop_distance) return 0.0;
  if (dist >= slowdown_distance) return 1.0;
  return (dist - stop_distance) / (slowdown_distance - stop_distance);
}

// Overall factor = min across all enabled sensors (1.0 if none enabled).
static double overallFactor(const ros::Time &now) {
  double f = 1.0;
  for (int i = 0; i < USS_COUNT; i++) {
    double sf = sensorFactor(i, now);
    if (sf < f) f = sf;
  }
  return f;
}

// Parse uss_input_config into uss_enabled[] using the same rule as
// mower_comms.cpp: a token is active if it contains 'A' (case-insensitive).
static void parseUssInputConfig(const std::string &config) {
  for (int i = 0; i < USS_COUNT; i++) uss_enabled[i] = false;

  size_t start = 0;
  int sensor_idx = 0;
  while (sensor_idx < USS_COUNT) {
    size_t comma = config.find(',', start);
    std::string token;
    if (comma == std::string::npos) {
      token = config.substr(start);
      start = config.size();
    } else {
      token = config.substr(start, comma - start);
      start = comma + 1;
    }
    bool active = false;
    for (size_t k = 0; k < token.size(); k++) {
      if (std::toupper(static_cast<unsigned char>(token[k])) == 'E') { active = true; break; }
    }
    uss_enabled[sensor_idx] = active;
    sensor_idx++;
    if (start >= config.size()) break;
  }
}

// ---- callbacks ----
static void onUss(const sensor_msgs::Range::ConstPtr &msg) {
  // parse the sensor index from frame_id "uss_<i>"
  const std::string &frame = msg->header.frame_id;
  if (frame.compare(0, 4, "uss_") != 0) return;
  int i = 0;
  try {
    i = std::stoi(frame.substr(4));
  } catch (...) {
    return;
  }
  if (i < 0 || i >= USS_COUNT) return;

  ros::Time now = ros::Time::now();

  // store the reading (header.stamp is the measurement time)
  uss_readings[i].range = msg->range;
  uss_readings[i].stamp = msg->header.stamp;

  // update the low-pass average (enabled sensors, readings below threshold only)
  if (uss_enabled[i] && std::isfinite(msg->range) && msg->range < uss_average_threshold) {
    uss_average += (static_cast<double>(uss_average_coef) / USS_FILT_SCALE) * (msg->range - uss_average);
  }

  // log the average every 10 s
  ROS_INFO_THROTTLE(10, "[uss_slowdown] low-pass US avg (readings < %.2f m) = %.3f m",
                    uss_average_threshold, uss_average);

  // gate: only override the autonomous command
  if (hl_state != mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS) return;
  if (cfg.sensor_behavior !=2) return;
  if ((now - last_nav_vel_time).toSec() > NAV_VEL_TIMEOUT) return;  // nav stale -> don't override

  double f = overallFactor(now);

  // debug: log the overall factor while the slowdown is active
  if (f < 1.0) {
    ROS_INFO_THROTTLE(0.5, "[uss_slowdown] active: overall slowdown factor = %.3f", f);
  }

  geometry_msgs::Twist out;
  if (f >= 1.0) {
    // clear: publish nothing so /nav_vel passes through at full speed
    return;
  } else if (f <= 0.0) {
    // full stop: publish all-zero every US message (must keep publishing)
    out = geometry_msgs::Twist();
  } else {
    out = last_nav_vel;
    out.linear.x  *= f;
    out.angular.z *= f;
  }
  uss_vel_pub.publish(out);
}

static void onNavVel(const geometry_msgs::Twist::ConstPtr &msg) {
  last_nav_vel = *msg;
  last_nav_vel_time = ros::Time::now();  // Twist has no header; use receive time
}

static void onState(const mower_msgs::HighLevelStatus::ConstPtr &msg) {
  hl_state = msg->state;
}

static void reconfigCB(const mower_logic::MowerLogicConfig &c) {
  cfg = c;
  parseUssInputConfig(c.uss_input_config);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "uss_slowdown");

  ros::NodeHandle n;
  ros::NodeHandle paramNh("~");

  if (paramNh.param("uss_front_slowdown_percent", uss_front_slowdown_percent, 0.75))
    ROS_INFO_STREAM("[uss_slowdown] uss_front_slowdown_percent = " << uss_front_slowdown_percent);
  if (paramNh.param("uss_side_slowdown_percent", uss_side_slowdown_percent, 0.50))
    ROS_INFO_STREAM("[uss_slowdown] uss_side_slowdown_percent = " << uss_side_slowdown_percent);
  if (paramNh.param("uss_front_stop_distance", uss_front_stop_distance, 0.15))
    ROS_INFO_STREAM("[uss_slowdown] uss_front_stop_distance = " << uss_front_stop_distance);
  if (paramNh.param("uss_side_stop_distance", uss_side_stop_distance, 0.10))
    ROS_INFO_STREAM("[uss_slowdown] uss_side_stop_distance = " << uss_side_stop_distance);
  if (paramNh.param("uss_average_threshold", uss_average_threshold, 1.0))
    ROS_INFO_STREAM("[uss_slowdown] uss_average_threshold = " << uss_average_threshold);
  if (paramNh.param("uss_average_coef", uss_average_coef, 218))
    ROS_INFO_STREAM("[uss_slowdown] uss_average_coef = " << uss_average_coef);

  uss_vel_pub = n.advertise<geometry_msgs::Twist>("/uss_vel", 1);

  ros::Subscriber uss_sub     = n.subscribe("/mower/uss", 5, onUss);
  ros::Subscriber nav_vel_sub = n.subscribe("/nav_vel", 10, onNavVel);
  ros::Subscriber state_sub   = n.subscribe("/mower_logic/current_state", 10, onState);

  // consume the mower_logic config (sensor_behavior + uss_input_config)
  cfg = mower_logic::MowerLogicConfig::__getDefault__();
  parseUssInputConfig(cfg.uss_input_config);
  reconfigClient = new dynamic_reconfigure::Client<mower_logic::MowerLogicConfig>("/mower_logic", reconfigCB);

  ROS_INFO("[uss_slowdown] started (waiting for mower_logic config)");

  ros::spin();

  return 0;
}
