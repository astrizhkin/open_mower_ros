// uss_slowdown_costmap: TTC-based ultrasonic slowdown for the autonomous drive.
//
// Complementary to uss_slowdown (raw per-sensor ranges). This node reads the
// fused uss_costmap grid and, on every grid update, measures the distance from
// the robot FOOTPRINT to the FIRST occupied cell in front of the direction of
// motion, then predicts the time-to-collision (TTC) with the current MEASURED
// velocity. When TTC < ttc_threshold it slows the autonomous command through
// the same twist_mux input as uss_slowdown (/uss_vel, priority 70) so the two
// nodes are mutually exclusive; a hard stop is issued when the in-front
// distance drops below stop_distance.
//
// "In front of motion": a cell that is (a) ahead of the footprint along the
// motion direction and (b) within a lateral corridor matching the footprint
// width (+ corridor_margin). Side/rear obstacles do not trigger a slowdown.
//
// Diagnostics published every update (so bags carry the trace): /uss_costmap_ttc
// (s) and /uss_costmap_front_dist (m), 1e3 = clear / not closing.
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
static double footprint_radius;   // m; fallback footprint half-extent if topic is stale
static double grid_timeout;       // s
static double footprint_timeout;  // s
static double measured_vel_timeout; // s
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

static struct { double x, y; } footprint_pts[128];
static int footprint_n = 0;
static ros::Time footprint_time;
static bool footprint_valid = false;

static double meas_vx = 0.0, meas_vy = 0.0;
static ros::Time measured_vel_time;
static bool measured_vel_valid = false;

static geometry_msgs::Twist last_nav_vel;
static ros::Time last_nav_vel_time;

static uint8_t hl_state = mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_NULL;
static mower_logic::MowerLogicConfig cfg;
static dynamic_reconfigure::Client<mower_logic::MowerLogicConfig> *reconfigClient;

static tf2_ros::Buffer *tf_buffer;
static tf2_ros::TransformListener *tf_listener;

static ros::Publisher uss_vel_pub;
static ros::Publisher ttc_pub;
static ros::Publisher front_dist_pub;

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

  // measured velocity (base_link)
  double vx = 0.0, vy = 0.0, v = 0.0;
  if (measured_vel_valid && (now - measured_vel_time).toSec() <= measured_vel_timeout) {
    vx = meas_vx; vy = meas_vy; v = std::hypot(vx, vy);
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

  // footprint corners in the global frame (topic, else a square around the pose)
  std::vector<Pt> corners;
  if (footprint_valid && footprint_n > 0 &&
      (now - footprint_time).toSec() <= footprint_timeout) {
    corners.reserve(footprint_n);
    for (int i = 0; i < footprint_n; ++i)
      corners.push_back({footprint_pts[i].x, footprint_pts[i].y});
  } else if (have_tf) {
    for (double sx : {-1.0, 1.0})
      for (double sy : {-1.0, 1.0}) {
        Pt p;
        p.x = px + sx * footprint_radius * std::cos(yaw) - sy * footprint_radius * std::sin(yaw);
        p.y = py + sx * footprint_radius * std::sin(yaw) + sy * footprint_radius * std::cos(yaw);
        corners.push_back(p);
      }
  }

  // distance + TTC
  double dist = CLEAR_SENTINEL;
  double ttc = CLEAR_SENTINEL;
  if (have_tf && v > min_speed && corners.size() >= 3) {
    const double phi = yaw + std::atan2(vy, vx);
    const double ux = std::cos(phi), uy = std::sin(phi);
    dist = frontDistance(grid, corners, ux, uy);
    ttc = (dist < CLEAR_SENTINEL) ? dist / v : CLEAR_SENTINEL;
  }

  // diagnostics (always, so bags carry the trace)
  std_msgs::Float32 ttc_msg;   ttc_msg.data = static_cast<float>(ttc);
  std_msgs::Float32 dist_msg;  dist_msg.data = static_cast<float>(dist);
  ttc_pub.publish(ttc_msg);
  front_dist_pub.publish(dist_msg);

  // gate: only override the autonomous command (same as uss_slowdown)
  if ((now - grid_time).toSec() > grid_timeout) return;                 // grid stale
  if (hl_state != mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS) return;
  if (cfg.sensor_behavior != 2) return;
  if ((now - last_nav_vel_time).toSec() > NAV_VEL_TIMEOUT) return;  // nav stale
  if (v < min_speed) return;                                        // not closing

  if (dist < stop_distance) {
    // HARD STOP — keep publishing so the mux holds it
    geometry_msgs::Twist stop;
    uss_vel_pub.publish(stop);
    ROS_INFO_THROTTLE(0.5, "[uss_slowdown_costmap] STOP: in-front dist %.2f m < %.2f m",
                      dist, stop_distance);
    return;
  }

  if (ttc < ttc_threshold) {
    // GRADED TAPER — scale the nav command so TTC is brought back to the threshold
    double target = dist / ttc_threshold;
    double s = std::min(1.0, target / std::max(last_nav_vel.linear.x, 1e-3));
    geometry_msgs::Twist out = last_nav_vel;
    out.linear.x  *= s;
    out.angular.z *= s;
    uss_vel_pub.publish(out);
    ROS_INFO_THROTTLE(0.5, "[uss_slowdown_costmap] taper: dist %.2f m ttc %.2f s -> scale %.2f",
                      dist, ttc, s);
  }
  // else clear: publish nothing -> twist_mux timeout releases /uss_vel
}

static void onGrid(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  grid = *msg;
  grid_time = msg->header.stamp;
  computeAndMaybeOverride();
}

static void onFootprint(const geometry_msgs::PolygonStamped::ConstPtr &msg)
{
  footprint_n = 0;
  const auto &pts = msg->polygon.points;
  for (size_t i = 0; i < pts.size() && footprint_n < 128; ++i) {
    footprint_pts[footprint_n].x = pts[i].x;
    footprint_pts[footprint_n].y = pts[i].y;
    ++footprint_n;
  }
  footprint_time = msg->header.stamp;
  footprint_valid = (footprint_n > 0);
}

static void onMeasuredVel(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  meas_vx = msg->twist.linear.x;
  meas_vy = msg->twist.linear.y;
  measured_vel_time = msg->header.stamp;
  measured_vel_valid = true;
}

static void onNavVel(const geometry_msgs::Twist::ConstPtr &msg)
{
  last_nav_vel = *msg;
  last_nav_vel_time = ros::Time::now();  // Twist has no header
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
    ROS_INFO_STREAM("[uss_slowdown_costmap] ttc_threshold = " << ttc_threshold);
  if (paramNh.param("stop_distance", stop_distance, 0.30))
    ROS_INFO_STREAM("[uss_slowdown_costmap] stop_distance = " << stop_distance);
  if (paramNh.param("min_speed", min_speed, 0.03))
    ROS_INFO_STREAM("[uss_slowdown_costmap] min_speed = " << min_speed);
  if (paramNh.param("occupied_threshold", occupied_threshold, 90))
    ROS_INFO_STREAM("[uss_slowdown_costmap] occupied_threshold = " << occupied_threshold);
  if (paramNh.param("corridor_margin", corridor_margin, 0.05))
    ROS_INFO_STREAM("[uss_slowdown_costmap] corridor_margin = " << corridor_margin);
  if (paramNh.param("footprint_radius", footprint_radius, 0.30))
    ROS_INFO_STREAM("[uss_slowdown_costmap] footprint_radius = " << footprint_radius);
  if (paramNh.param("grid_timeout", grid_timeout, 0.5))
    ROS_INFO_STREAM("[uss_slowdown_costmap] grid_timeout = " << grid_timeout);
  if (paramNh.param("footprint_timeout", footprint_timeout, 0.5))
    ROS_INFO_STREAM("[uss_slowdown_costmap] footprint_timeout = " << footprint_timeout);
  if (paramNh.param("measured_vel_timeout", measured_vel_timeout, 0.3))
    ROS_INFO_STREAM("[uss_slowdown_costmap] measured_vel_timeout = " << measured_vel_timeout);
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

  ros::Subscriber grid_sub   = n.subscribe(grid_topic, 1, onGrid);
  ros::Subscriber fp_sub     = n.subscribe(footprint_topic, 5, onFootprint);
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
