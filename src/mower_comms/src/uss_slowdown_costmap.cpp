// uss_slowdown_costmap: TTC-based ultrasonic slowdown for the autonomous drive.
//
// Complementary to uss_slowdown (raw per-sensor ranges). This node reads the
// fused uss_costmap grid and, on every grid update, measures the distance from
// the robot FOOTPRINT to the FIRST occupied cell in front of the robot, then
// predicts the time-to-collision (TTC) with the current MEASURED velocity
// (forward motion only — the robot has no rear sensors). When TTC <
// ttc_threshold it slows the autonomous command through the same twist_mux
// input as uss_slowdown (/uss_vel, priority 70) so the two
// nodes are mutually exclusive; a hard stop is issued when the in-front
// distance drops below stop_distance.
//
// "In front": the robot forward direction, within a lateral corridor matching
// the footprint width (+ corridor_margin). Side/rear obstacles do not trigger
// a slowdown; backward motion is never checked.
//
// Footprint source (global frame): primarily the uss_costmap's transformed
// footprint topic (~footprint_topic) — the costmap re-transforms and
// republishes it every cycle with the current robot pose, so the node tracks
// whatever footprint the costmap is configured with, no re-anchoring needed.
// Falls back to the robot-frame rectangle from the ~robot_front/~robot_rear/
// ~robot_width params placed at the current TF pose while the topic is
// missing, stale (~footprint_timeout), or degenerate.
//
// Diagnostics published every update (so bags carry the trace): /uss_costmap_ttc
// (s) and /uss_costmap_front_dist (m), 1e3 = clear / not closing.
// /uss_slowdown_costmap/local_footprint (PolygonStamped, global frame) is the
// node's own param footprint, always published for RViz overlay against the
// costmap's footprint topic.
//
// Gate (same as uss_slowdown): override only in HIGH_LEVEL_STATE_AUTONOMOUS with
// mower_logic sensor_behavior == 2 and a fresh /nav_vel.
//
// Modeled on uss_slowdown.cpp (same package).

#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Float32.h>
#include <mower_msgs/HighLevelStatus.h>
#include <dynamic_reconfigure/client.h>
#include "mower_logic/MowerLogicConfig.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <vector>

// ---- tuning params (private) ----
static double ttc_threshold;      // s; slow down below this TTC
static double stop_distance;      // m; hard stop when in-front distance < this
static double min_speed;          // m/s; below this the robot is not "closing"
static int    occupied_threshold; // grid value >= this counts as occupied
static double corridor_margin;    // m; extra lateral half-width around the footprint
static double grid_timeout;       // s
static double measured_vel_timeout; // s
static double footprint_timeout;  // s; stale costmap footprint -> param fallback
static double robot_front;        // m; front edge distance from the base_link origin
static double robot_rear;         // m; rear edge distance from the base_link origin
static double robot_width;        // m; total robot width
static const  double NAV_VEL_TIMEOUT = 2.0;  // s (nav publishes ~1 Hz)

static std::string grid_topic;
static std::string footprint_topic;
static std::string measured_vel_topic;
static std::string nav_vel_topic;
static std::string global_frame = "map";
static std::string base_frame   = "base_link";

static const double CLEAR_SENTINEL = 1000.0;  // "no obstacle in front"

// ---- state ----
static nav_msgs::OccupancyGrid grid;          // latest grid
static ros::Time grid_time;

static double meas_vx = 0.0;
static ros::Time measured_vel_time;
static bool measured_vel_valid = false;

static geometry_msgs::Twist last_nav_vel;
static ros::Time last_nav_vel_time;

static geometry_msgs::PolygonStamped footprint_msg;  // latest costmap footprint (map frame)
static ros::Time footprint_time;
static bool have_footprint = false;

static uint8_t hl_state = mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_NULL;
static mower_logic::MowerLogicConfig cfg;
static dynamic_reconfigure::Client<mower_logic::MowerLogicConfig> *reconfigClient;

static tf2_ros::Buffer *tf_buffer;
static tf2_ros::TransformListener *tf_listener;

static ros::Publisher uss_vel_pub;
static ros::Publisher ttc_pub;
static ros::Publisher front_dist_pub;
static ros::Publisher local_footprint_pub;

struct Pt { double x, y; };

// Distance from the footprint to the nearest occupied cell in front of motion,
// or CLEAR_SENTINEL if none. `corners` = footprint corners in the global frame,
// `motion` = unit direction of travel in the global frame.
static double frontDistance(const nav_msgs::OccupancyGrid &g,
                            const std::vector<Pt> &corners,
                            double ux, double uy)
{
  if (corners.empty()) return CLEAR_SENTINEL;

  double cx = 0.0, cy = 0.0;
  for (const auto &p : corners) { cx += p.x; cy += p.y; }
  cx /= static_cast<double>(corners.size());
  cy /= static_cast<double>(corners.size());

  const double nx = -uy, ny = ux;  // left normal of the motion direction

  double front_edge = -1e9;
  double lat_min = 1e9, lat_max = -1e9;
  for (const auto &p : corners) {
    double rx = p.x - cx, ry = p.y - cy;
    front_edge = std::max(front_edge, rx * ux + ry * uy);
    double lat = rx * nx + ry * ny;
    lat_min = std::min(lat_min, lat);
    lat_max = std::max(lat_max, lat);
  }
  double half_w = 0.5 * (lat_max - lat_min) + corridor_margin;

  const double res = g.info.resolution;
  const double ox = g.info.origin.position.x;
  const double oy = g.info.origin.position.y;
  const int W = static_cast<int>(g.info.width);
  const int H = static_cast<int>(g.info.height);
  const auto &d = g.data;

  double best = CLEAR_SENTINEL;
  for (int r = 0; r < H; ++r) {
    const double wy = oy + (r + 0.5) * res;
    const double ry = wy - cy;
    const double lat_r = ry * ny;                    // row-common part of lat
    const int row_off = r * W;
    for (int c = 0; c < W; ++c) {
      const int8_t v = d[row_off + c];
      if (v < occupied_threshold) continue;
      const double wx = ox + (c + 0.5) * res;
      const double rx = wx - cx;
      const double a = rx * ux + ry * uy;            // along motion
      if (a <= front_edge) continue;                 // not in front
      const double lat = rx * nx + lat_r;            // perpendicular
      if (fabs(lat) > half_w) continue;              // outside corridor
      const double dist = a - front_edge;
      if (dist < best) best = dist;
    }
  }
  return best;
}

static void computeAndMaybeOverride()
{
  ros::Time now = ros::Time::now();
  if (now == ros::Time(0)) return;  // sim time not started yet

  // measured velocity (base_link). Differential drive: the publisher fills
  // linear.x and angular.z only. The slowdown acts on forward motion only.
  double vx = 0.0;
  if (measured_vel_valid && (now - measured_vel_time).toSec() <= measured_vel_timeout) {
    vx = meas_vx;
  }

  // robot pose in the global frame (position + yaw)
  double px = 0.0, py = 0.0, yaw = 0.0;
  bool have_tf = false;
  try {
    geometry_msgs::TransformStamped t =
        tf_buffer->lookupTransform(global_frame, base_frame, ros::Time(0));
    px = t.transform.translation.x; py = t.transform.translation.y;
    yaw = tf2::getYaw(t.transform.rotation);
    have_tf = true;
  } catch (std::exception &) {
    have_tf = false;
  }

  // local (param) footprint: the robot-frame rectangle from
  // ~robot_front/~robot_rear/~robot_width at the current TF pose. It is the
  // fallback footprint source, and it is ALWAYS published on
  // /uss_slowdown_costmap/local_footprint (same format as the costmap's
  // footprint) so RViz can overlay it on the costmap layer for comparison.
  std::vector<Pt> local_corners;
  if (have_tf) {
    const double cw = std::cos(yaw), sw = std::sin(yaw);
    const double hw = 0.5 * robot_width;
    auto place = [&, cw, sw](double lx, double ly) {
      local_corners.push_back({px + lx * cw - ly * sw, py + lx * sw + ly * cw});
    };
    place(robot_front, hw);
    place(robot_front, -hw);
    place(-robot_rear, -hw);
    place(-robot_rear, hw);
  }
  if (!local_corners.empty()) {
    geometry_msgs::PolygonStamped local_fp;
    local_fp.header.stamp = now;
    local_fp.header.frame_id = global_frame;
    for (const auto &p : local_corners) {
      geometry_msgs::Point32 pt;
      pt.x = p.x; pt.y = p.y;
      local_fp.polygon.points.push_back(pt);
    }
    local_footprint_pub.publish(local_fp);
  }

  // footprint used for the forward corridor. PRIMARY: the costmap's
  // transformed footprint topic (fresh, non-degenerate, right frame) — the
  // same pose the grid was built from, republished every costmap cycle.
  // FALLBACK: the local rectangle above.
  std::vector<Pt> corners;
  bool from_topic = false;
  if (have_footprint &&
      (now - footprint_time).toSec() <= footprint_timeout &&
      footprint_msg.header.frame_id == global_frame &&
      footprint_msg.polygon.points.size() >= 3) {
    // a zero-area polygon (all points collapsed) would give a degenerate
    // corridor — treat it as unusable
    double area = 0.0;
    const auto &pts = footprint_msg.polygon.points;
    for (size_t i = 0; i < pts.size(); ++i) {
      const auto &p = pts[i];
      const auto &q = pts[(i + 1) % pts.size()];
      area += p.x * q.y - q.x * p.y;
    }
    if (fabs(area) > 1e-6) {
      from_topic = true;
      for (const auto &p : pts) corners.push_back({p.x, p.y});
    }
  }
  if (!from_topic) corners = local_corners;

  // log the initial source and every switch (rare; guarded against
  // flapping at the timeout edge) — a silently running fallback is bad
  static bool src_logged = false;
  static bool last_from_topic = false;
  static ros::Time last_src_log;
  if ((!src_logged || from_topic != last_from_topic) &&
      now - last_src_log > ros::Duration(1.0)) {
    if (from_topic)
      ROS_INFO("[uss_slowdown_costmap] footprint source: costmap topic (%zu points)",
               footprint_msg.polygon.points.size());
    else
      ROS_INFO("[uss_slowdown_costmap] footprint source: local params (topic %s)",
               have_footprint ? "stale/degenerate" : "not received");
    src_logged = true;
    last_from_topic = from_topic;
    last_src_log = now;
  }

  // distance to the first occupied cell in front of the robot (forward only —
  // there are no rear sensors, so backward motion is never checked). Reported
  // whenever we have geometry, at any speed (the low-speed hard stop below
  // relies on it). TTC: only while moving forward faster than min_speed.
  double dist = CLEAR_SENTINEL;
  double ttc = CLEAR_SENTINEL;
  if (have_tf && corners.size() >= 3) {
    const double ux = std::cos(yaw), uy = std::sin(yaw);
    dist = frontDistance(grid, corners, ux, uy);
    ttc = (dist < CLEAR_SENTINEL && vx > min_speed) ? dist / vx : CLEAR_SENTINEL;
  }

  // diagnostics (always, so bags carry the trace)
  std_msgs::Float32 ttc_msg;   ttc_msg.data = static_cast<float>(ttc);
  std_msgs::Float32 dist_msg;  dist_msg.data = static_cast<float>(dist);
  ttc_pub.publish(ttc_msg);
  front_dist_pub.publish(dist_msg);

  bool run_comand = true;

  // gate: only override the autonomous command (same as uss_slowdown)
  if ((now - grid_time).toSec() > grid_timeout) return;                 // grid stale
  if (hl_state != mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS) run_comand = false;
  if (cfg.sensor_behavior != 2) run_comand = false;
  if ((now - last_nav_vel_time).toSec() > NAV_VEL_TIMEOUT) return;  // nav stale

  if (dist < stop_distance) {
    // HARD STOP — keep publishing so the mux holds it
    if(run_comand) {
      geometry_msgs::Twist stop;
      uss_vel_pub.publish(stop);
    }
    ROS_INFO_THROTTLE(0.5, "[uss_slowdown_costmap] %sSTOP: in-front dist %.2f m < %.2f m",
                      run_comand ? "" : "SIM ", dist, stop_distance);
    return;
  }

  if (vx <= min_speed) return;                    // not closing forward

  if (ttc < ttc_threshold) {
    // GRADED TAPER — scale the nav command so TTC is brought back to the threshold
    double target = dist / ttc_threshold;
    double s = std::min(1.0, target / std::max(std::fabs(last_nav_vel.linear.x), 1e-3));
    geometry_msgs::Twist out = last_nav_vel;
    out.linear.x  *= s;
    out.angular.z *= s;
    if(run_comand) {
      uss_vel_pub.publish(out);
    }
    ROS_INFO_THROTTLE(0.5, "[uss_slowdown_costmap] %staper: dist %.2f m ttc %.2f s -> scale %.2f",
                      run_comand ? "" : "SIM ", dist, ttc, s);
  }
  // else clear: publish nothing -> twist_mux timeout releases /uss_vel
}

static void onGrid(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  grid = *msg;
  grid_time = msg->header.stamp;
  computeAndMaybeOverride();
}

static void onMeasuredVel(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  // differential drive: only linear.x (and angular.z) are ever filled
  meas_vx = msg->twist.linear.x;
  measured_vel_time = msg->header.stamp;
  measured_vel_valid = true;
}

static void onNavVel(const geometry_msgs::Twist::ConstPtr &msg)
{
  last_nav_vel = *msg;
  last_nav_vel_time = ros::Time::now();  // Twist has no header
}

static void onFootprint(const geometry_msgs::PolygonStamped::ConstPtr &msg)
{
  footprint_msg = *msg;
  footprint_time = msg->header.stamp;
  have_footprint = true;
}

static void onState(const mower_msgs::HighLevelStatus::ConstPtr &msg)
{
  hl_state = msg->state;
}

static void reconfigCB(const mower_logic::MowerLogicConfig &c)
{
  cfg = c;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uss_slowdown_costmap");

  ros::NodeHandle n;
  ros::NodeHandle paramNh("~");

  if (paramNh.param("ttc_threshold", ttc_threshold, 1.0))
    ROS_INFO_STREAM("[uss_slowdown_costmap] ttc_threshold sec = " << ttc_threshold);
  if (paramNh.param("stop_distance", stop_distance, 0.30))
    ROS_INFO_STREAM("[uss_slowdown_costmap] stop_distance = " << stop_distance);
  if (paramNh.param("min_speed", min_speed, 0.05))
    ROS_INFO_STREAM("[uss_slowdown_costmap] min_speed = " << min_speed);
  if (paramNh.param("occupied_threshold", occupied_threshold, 90))
    ROS_INFO_STREAM("[uss_slowdown_costmap] occupied_threshold = " << occupied_threshold);
  if (paramNh.param("corridor_margin", corridor_margin, 0.1))
    ROS_INFO_STREAM("[uss_slowdown_costmap] corridor_margin = " << corridor_margin);
  if (paramNh.param("grid_timeout", grid_timeout, 0.5))
    ROS_INFO_STREAM("[uss_slowdown_costmap] grid_timeout = " << grid_timeout);
  if (paramNh.param("footprint_timeout", footprint_timeout, 0.5))
    ROS_INFO_STREAM("[uss_slowdown_costmap] footprint_timeout = " << footprint_timeout);
  if (paramNh.param("measured_vel_timeout", measured_vel_timeout, 0.3))
    ROS_INFO_STREAM("[uss_slowdown_costmap] measured_vel_timeout = " << measured_vel_timeout);
  if (paramNh.param("robot_front", robot_front, 0.39))
    ROS_INFO_STREAM("[uss_slowdown_costmap] robot_front = " << robot_front);
  if (paramNh.param("robot_rear", robot_rear, 0.33))
    ROS_INFO_STREAM("[uss_slowdown_costmap] robot_rear = " << robot_rear);
  if (paramNh.param("robot_width", robot_width, 0.66))
    ROS_INFO_STREAM("[uss_slowdown_costmap] robot_width = " << robot_width);
  if (paramNh.param("grid_topic", grid_topic, std::string("/uss_costmap/costmap/costmap")))
    ROS_INFO_STREAM("[uss_slowdown_costmap] grid_topic = " << grid_topic);
  if (paramNh.param("footprint_topic", footprint_topic, std::string("/uss_costmap/costmap/footprint")))
    ROS_INFO_STREAM("[uss_slowdown_costmap] footprint_topic = " << footprint_topic);
  if (paramNh.param("measured_vel_topic", measured_vel_topic, std::string("/mower/measured_vel")))
    ROS_INFO_STREAM("[uss_slowdown_costmap] measured_vel_topic = " << measured_vel_topic);
  if (paramNh.param("nav_vel_topic", nav_vel_topic, std::string("/nav_vel")))
    ROS_INFO_STREAM("[uss_slowdown_costmap] nav_vel_topic = " << nav_vel_topic);
  if (paramNh.param("global_frame", global_frame, std::string("map")))
    ROS_INFO_STREAM("[uss_slowdown_costmap] global_frame = " << global_frame);
  if (paramNh.param("base_frame", base_frame, std::string("base_link")))
    ROS_INFO_STREAM("[uss_slowdown_costmap] base_frame = " << base_frame);

  uss_vel_pub       = n.advertise<geometry_msgs::Twist>("/uss_vel", 1);
  ttc_pub           = n.advertise<std_msgs::Float32>("/uss_costmap_ttc", 10);
  front_dist_pub    = n.advertise<std_msgs::Float32>("/uss_costmap_front_dist", 10);
  local_footprint_pub = n.advertise<geometry_msgs::PolygonStamped>("/uss_slowdown_costmap/local_footprint", 10);

  ros::Subscriber grid_sub   = n.subscribe(grid_topic, 1, onGrid);
  ros::Subscriber foot_sub   = n.subscribe(footprint_topic, 1, onFootprint);
  ros::Subscriber meas_sub   = n.subscribe(measured_vel_topic, 5, onMeasuredVel);
  ros::Subscriber nav_sub    = n.subscribe(nav_vel_topic, 10, onNavVel);
  ros::Subscriber state_sub  = n.subscribe("/mower_logic/current_state", 10, onState);

  // mower_logic config (sensor_behavior gate), same as uss_slowdown
  cfg = mower_logic::MowerLogicConfig::__getDefault__();
  reconfigClient = new dynamic_reconfigure::Client<mower_logic::MowerLogicConfig>("/mower_logic", reconfigCB);

  tf_buffer  = new tf2_ros::Buffer(ros::Duration(10.0));
  tf_listener = new tf2_ros::TransformListener(*tf_buffer);

  ROS_INFO("[uss_slowdown_costmap] started (waiting for grid + measured vel + TF)");

  ros::spin();

  return 0;
}
