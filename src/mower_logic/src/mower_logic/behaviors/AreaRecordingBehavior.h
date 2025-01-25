// Created by Clemens Elflein on 2/21/22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based
// on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
#ifndef SRC_AREA_RECORDING_BEHAVIOR_H
#define SRC_AREA_RECORDING_BEHAVIOR_H

#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mower_map/GetDockingPointSrv.h>
#include <tf2/LinearMath/Transform.h>

#include "Behavior.h"
#include "DockingBehavior.h"
#include "IdleBehavior.h"
#include "geometry_msgs/Twist.h"
#include "mower_map/AddMowingAreaSrv.h"
#include "mower_map/MapArea.h"
#include "mower_map/MapAreas.h"
#include "mower_map/SetDockingPointSrv.h"
#include "mower_msgs/EmergencyModeSrv.h"
#include "mower_msgs/Status.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "xbot_msgs/AbsolutePose.h"
#include "xbot_msgs/MapOverlay.h"

#define NEW_POINT_MIN_DISTANCE 0.1

class AreaRecordingBehavior : public Behavior {
 public:
  static AreaRecordingBehavior INSTANCE;

  AreaRecordingBehavior();

 private:
  bool has_odom = false;

  sensor_msgs::Joy last_joy;
  xbot_msgs::AbsolutePose last_pose;

  ros::Publisher map_overlay_pub;
  ros::Publisher marker_pub;
  ros::Publisher marker_array_pub;

  ros::Subscriber joy_sub, pose_sub;

  ros::Subscriber dock_sub, polygon_sub, mow_area_sub, nav_area_sub, auto_point_collecting_sub, collect_point_sub;

  ros::ServiceClient add_mowing_area_client, set_docking_point_client;

  bool has_first_docking_pos = false;
  geometry_msgs::Pose first_docking_pos;

  // true, if we should be recording the current data into a polygon
  bool poly_recording_enabled = false;

  // true, if all polys were recorded and the complete area is finished
  bool finished_all = false;
  uint8_t area_type = 0;
  bool set_docking_position = false;

  // auto point collecting enabled to true points are collected automatically
  // if distance is greater than NEW_POINT_MIN_DISTANCE during recording
  // otherwise collect_point has to be set to true manually for each point to be recorded
  bool auto_point_collecting = true;
  bool collect_point = false;

  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;

 private:
  bool recordNewPolygon(geometry_msgs::Polygon &polygon, xbot_msgs::MapOverlay &resultOverlay);
  bool getDockingPosition(geometry_msgs::Pose &pos);
  void pose_received(const xbot_msgs::AbsolutePose::ConstPtr &msg);
  void joy_received(const sensor_msgs::Joy &joy_msg);
  void record_dock_received(std_msgs::Bool state_msg);
  void record_polygon_received(std_msgs::Bool state_msg);
  void record_mowing_received(std_msgs::Bool state_msg);
  void record_prohibited_received(std_msgs::Bool state_msg);
  void record_navigation_received(std_msgs::Bool state_msg);
  void record_auto_point_collecting(std_msgs::Bool state_msg);
  void record_collect_point(std_msgs::Bool state_msg);

  void finish_area(uint8_t areaType, std::string reason);
  void update_actions();

 public:
  std::string state_name() override;

  std::string sub_state_name() override;

  Behavior *execute() override;

  void enter() override;

  void exit() override;

  void reset() override;

  bool needs_gps() override;

  void command_home() override;

  void command_start() override;

  void command_s1() override;

  void command_s2() override;

  bool redirect_joystick() override;

  uint8_t get_sub_state() override;

  uint8_t get_state() override;

  void handle_action(std::string action) override;
};

#endif  // SRC_AREA_RECORDING_BEHAVIOR_H
