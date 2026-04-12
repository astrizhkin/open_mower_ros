//
// Created by Clemens Elflein on 15.03.22.
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

//#define WHEEL_TICKS_MSG
//#define HOVERBOARD_ODOM

#include <dynamic_reconfigure/client.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mower_msgs/Status.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <contact_sensor_layer/Contact.h>

#include <algorithm>
#include <bitset>

#include "mower_logic/MowerLogicConfig.h"
#include "mower_msgs/EmergencyModeSrv.h"
#include "mower_msgs/HighLevelControlSrv.h"
#include "mower_msgs/HighLevelStatus.h"
#include "mower_msgs/MowerControlSrv.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "xbot_positioning/SetPoseSrv.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


ros::ServiceClient positioningClient;
ros::Publisher status_pub;
ros::Publisher sensor_imu_pub;
ros::Publisher uss_pub;
ros::Publisher contact_pub;

ros::Publisher cmd_vel_safe_pub;
ros::Publisher measured_vel_pub;

// 8 bits of high level emergency set/reset in ROS
uint8_t emergency_high_level_bits = 0;
std::string emergency_high_level_reasons[] = {"", "", "", "", "", "", "", ""};
ros::Time emergency_high_level_end[] = {ros::Time::ZERO, ros::Time::ZERO, ros::Time::ZERO, ros::Time::ZERO,
                                        ros::Time::ZERO, ros::Time::ZERO, ros::Time::ZERO, ros::Time::ZERO};

// True, if the LL board thinks there should be an emergency
uint8_t emergency_low_level_bits = 0;

mower_msgs::HighLevelStatus last_high_level_status;

// Current speeds (duty cycle) for the three ESCs
geometry_msgs::Twist last_cmd_twist;
ros::Time last_cmd_twist_time(0.0);
geometry_msgs::Twist last_cmd_vel_safe;
float speed_mow = 0;
uint8_t mower_enabled = 0;
uint8_t mower_direction = 0;

// Ticks / m and wheel distance for this robot
double wheel_radius_m = 0.0;
double wheel_separation_m = 0.0;
double cmd_vel_timout = 0.0;
bool publish_mag = false;

bool mower_esc_enabled = false;

// LL/HL configuration

// mower_msgs::HighLevelStatus last_high_level_status;
// ros::Time last_high_level_status_time(0.0);

sensor_msgs::Imu sensor_imu_msg;

ros::ServiceClient highLevelClient;

void sendWheelTickAndMeasuredTwist(ros::Time& stamp);

bool isEmergency() {
  return emergency_high_level_bits > 0 || emergency_low_level_bits > 0;
}

bool isTemporaryEmergency() {
  for (uint8_t bit = 0; bit < 8; bit++) {
    if (emergency_high_level_bits & (1 << bit) && emergency_high_level_end[bit].isZero()) {
      return false;
    }
  }
  return true && (emergency_low_level_bits == 1<<6 || emergency_low_level_bits == 0);
}

void updateEmergencyBits() {
  for (uint8_t bit = 0; bit < 8; bit++) {
    if (emergency_high_level_bits & (1 << bit) && !emergency_high_level_end[bit].isZero() &&
        emergency_high_level_end[bit] < ros::Time::now()) {
      ROS_WARN_STREAM("[mower_comms_sim] Autoreset emergency bit " << bit << " having reason "
                                                               << emergency_high_level_reasons[bit]);
      emergency_high_level_bits &= ~(1 << bit);
      emergency_high_level_end[bit] = ros::Time::ZERO;
    }
  }
}

void publishActuators() {
  geometry_msgs::Twist execute_vel;
  execute_vel.linear.x = last_cmd_twist.linear.x;
  execute_vel.angular.z = last_cmd_twist.angular.z;
  // emergency or timeout -> send 0 speeds
  if (isEmergency()) {
    // TODO: publish speed topic?
    execute_vel.linear.x = 0;
    execute_vel.angular.z = 0;
    speed_mow = 0;
  }
  if (ros::Time::now() - last_cmd_twist_time > ros::Duration(cmd_vel_timout)) {
    // TODO: publish speed topic?
    execute_vel.linear.x = 0;
    execute_vel.angular.z = 0;
  }
  if (ros::Time::now() - last_cmd_twist_time > ros::Duration(25.0)) {
    // TODO: publish speed topic?
    execute_vel.linear.x = 0;
    execute_vel.angular.z = 0;
    speed_mow = 0;
  }

  last_cmd_vel_safe = execute_vel;
  cmd_vel_safe_pub.publish(execute_vel);
}

void fillEscStatus(mower_msgs::ESCStatus &status) {
  status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
  status.temperature_pcb = 45;
  status.temperature_motor = 40;
  status.rpm = 0;
  status.tacho = 0;
  status.current = 0;
}

void publishStatus() {
  mower_msgs::Status status_msg;
  status_msg.stamp = ros::Time::now();

  status_msg.mower_status = mower_msgs::Status::MOWER_STATUS_OK;

  status_msg.raspberry_pi_power = true;
  status_msg.charging = false;
  //simulate low level board reaction to high level status IDLE
  status_msg.esc_power = last_high_level_status.state != mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_IDLE;
  status_msg.rain_detected = false;
  status_msg.uss_timeout = false;
  status_msg.imu_timeout = false;
  status_msg.battery_empty = false;
  status_msg.bms_timeout = false;
  status_msg.ll_timeout = false;
  status_msg.mow_enabled = true;

  for (int i = 0; i < 5; i++) {
    sensor_msgs::Range range_msg;
    range_msg.header.stamp = ros::Time::now();
    std::ostringstream uss_frame_id;
    uss_frame_id << "uss_" << i;
    range_msg.header.frame_id=uss_frame_id.str();
    range_msg.range = 2.5;//fake uss range
    range_msg.field_of_view = 60 * M_PI / 180;
    range_msg.min_range=0.0;
    range_msg.max_range=2.55;
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    uss_pub.publish(range_msg);
  }

  for (int i=0;i<4;i++) {
    contact_sensor_layer::Contact contact_msg;
    contact_msg.header.stamp = ros::Time::now();
    std::ostringstream contact_frame_id;
    contact_frame_id << "contact_" << i;
    contact_msg.header.frame_id=contact_frame_id.str();
    contact_msg.is_active = false;
    contact_pub.publish(contact_msg);
  }

  // overwrite emergency with the LL value.
  emergency_low_level_bits = 0;
  if (emergency_high_level_bits > 0) {
    ROS_ERROR_STREAM_THROTTLE(1, "[mower_comms_sim] High Level Emergency. Bitmask: " << (int)emergency_high_level_bits);
  }

  // True, if high or low level emergency condition is present
  status_msg.emergency = isEmergency();
  status_msg.temporary_emergency = isTemporaryEmergency();

  status_msg.v_battery = 39.0;
  status_msg.v_charge = 0.0;
  status_msg.battery_current = 0.0;
  status_msg.battery_soc = 95;
  status_msg.battery_temperature=20.0;
  status_msg.balancer_temperature=20.0;

  fillEscStatus(status_msg.front_left_esc_status);
  fillEscStatus(status_msg.rear_left_esc_status);
  fillEscStatus(status_msg.front_right_esc_status);
  fillEscStatus(status_msg.rear_right_esc_status);
  fillEscStatus(status_msg.mow_esc_status);

  // publis topic status
  status_pub.publish(status_msg);

  sendWheelTickAndMeasuredTwist(status_msg.stamp);

}

void sendWheelTickAndMeasuredTwist(ros::Time& stamp) {
  geometry_msgs::TwistStamped measured_wheel_tick_twist;
  measured_wheel_tick_twist.header.frame_id = "base_link";
  measured_wheel_tick_twist.header.stamp = stamp;
  measured_wheel_tick_twist.twist.linear.x = last_cmd_vel_safe.linear.x;
  measured_wheel_tick_twist.twist.angular.z = last_cmd_vel_safe.angular.z;

  measured_vel_pub.publish(measured_wheel_tick_twist);
}

void publishActuatorsTimerTask(const ros::TimerEvent &timer_event) {
  updateEmergencyBits();
  publishActuators();
  publishStatus();
}

bool setMowEnabled(mower_msgs::MowerControlSrvRequest &req, mower_msgs::MowerControlSrvResponse &res) {
  if (req.mow_enabled != mower_enabled) {
    ROS_INFO_STREAM("[mower_comms_sim] setMowEnabled(en=" << static_cast<unsigned>(req.mow_enabled)
                                                      << ", dir=" << static_cast<unsigned>(req.mow_direction) << ")");
  }
  mower_enabled = req.mow_enabled;
  mower_direction = req.mow_direction;
  if (mower_enabled && !isEmergency()) {
    speed_mow = req.mow_direction ? req.mow_power : -req.mow_power;
  } else {
    speed_mow = 0;
  }
  //    ROS_INFO_STREAM("[mower_comms_sim] Setting mow enabled to " << speed_mow);
  return true;
}

bool setEmergencyMode(mower_msgs::EmergencyModeSrvRequest &req, mower_msgs::EmergencyModeSrvResponse &res) {
  uint8_t emergency_bit = req.emergency_bit;
  uint8_t emergency_code = 1 << emergency_bit;
  if (emergency_bit == mower_msgs::EmergencyModeSrvRequest::EMERGENCY_ALL) {
    ROS_WARN_STREAM("[mower_comms_sim] Emergency ALL action");
    for (uint8_t bit = 0; bit < 8; bit++) {
      req.emergency_bit = bit;
      setEmergencyMode(req, res);
    }
    req.emergency_bit = mower_msgs::EmergencyModeSrvRequest::EMERGENCY_ALL;
    return true;
  }
  if (req.set_reset) {
    if (emergency_high_level_bits & emergency_code && emergency_high_level_end[emergency_bit].isZero() &&
        !req.duration.isZero()) {
      // active high level emergency has infinite duration, do not override it
      ROS_WARN_STREAM("[mower_comms_sim] Do not overide previous inifinite duration emergency with reason [" << req.reason
                                                                                                         << "]");
    } else {
      ROS_ERROR_STREAM("[mower_comms_sim] Setting emergency bit " << (int)emergency_bit << " with reason [" << req.reason
                                                              << "] duration " << req.duration.toSec());
      emergency_high_level_reasons[emergency_bit] = req.reason;
      emergency_high_level_end[emergency_bit] =
          req.duration.isZero() ? ros::Time::ZERO : ros::Time::now() + req.duration;
    }
    emergency_high_level_bits |= emergency_code;
  } else {
    if (emergency_high_level_bits & emergency_code) {
      ROS_WARN_STREAM("[mower_comms_sim] Clear emergency bit " << (int)emergency_bit << " with reason [" << req.reason
                                                           << "]");
      emergency_high_level_reasons[emergency_bit] = req.reason;
      emergency_high_level_end[emergency_bit] = ros::Time::ZERO;
    }
    emergency_high_level_bits &= ~emergency_code;
  }

  // Set the high level emergency instantly. Low level value will be set on next update.
  // High level emergency already set with unlimited duration

  publishActuators();
  return true;
}

void highLevelStatusReceived(const mower_msgs::HighLevelStatus::ConstPtr &msg) {
  // ROS_INFO_STREAM("[mower_comms_sim] High level status received: "<< msg->state_name << "/" << msg->sub_state_name);
  last_high_level_status = *msg;
}

void onCmdVelReceived(const geometry_msgs::Twist::ConstPtr &msg) {
  // TODO: update this to rad/s values and implement xESC speed control
  // ROS_INFO_STREAM("[mower_comms_sim] Got Twist: "<< +msg->linear.x << " " << +msg->angular.z);
  last_cmd_twist = *msg;
  last_cmd_twist_time = ros::Time::now();
}

void handleLowLevelIMU() {
  // imu_msg.dt = imu->dt_millis;
  sensor_imu_msg.header.stamp = ros::Time::now();
  sensor_imu_msg.header.seq++;
  sensor_imu_msg.header.frame_id = "base_link";
  sensor_imu_msg.linear_acceleration.x = 0.0;
  sensor_imu_msg.linear_acceleration.y = 0.0;
  sensor_imu_msg.linear_acceleration.z = 9.8;
  sensor_imu_msg.angular_velocity.x = 0.0;
  sensor_imu_msg.angular_velocity.y = 0.0;
  sensor_imu_msg.angular_velocity.z = last_cmd_vel_safe.angular.z;

  sensor_imu_pub.publish(sensor_imu_msg);
}

void setPose(double x, double y, double z,double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = tf2::toMsg(q);

  xbot_positioning::SetPoseSrv pose_srv;
  pose_srv.request.robot_pose = pose;
  pose_srv.request.reason = "simulation";

  ros::Rate retry_delay(1);
  bool success = false;
  for (int i = 0; i < 10; i++) {
    if (positioningClient.exists()){
      if (positioningClient.call(pose_srv)) {
        //            ROS_INFO_STREAM("successfully set pose to " << pose);
        success = true;
        break;
      }
      ROS_ERROR_STREAM("[mower_comms_sim] Error setting robot pose to " << pose << ". Retrying.");
    }else{
      ROS_INFO_STREAM("[mower_comms_sim] Waiting for robot pose service");
    }
    retry_delay.sleep();
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "mower_comms");

  ros::NodeHandle n;
  ros::NodeHandle paramNh("~");
  ros::NodeHandle mowerParamNh("~/mower_xesc");

  highLevelClient = n.serviceClient<mower_msgs::HighLevelControlSrv>("mower_service/high_level_control");

  if (!paramNh.getParam("wheel_radius_m", wheel_radius_m)) {
    ROS_ERROR_STREAM("[mower_comms_sim] Wheel radius must be specified for odometry. Quitting.");
    return 1;
  }

  if (!paramNh.getParam("wheel_separation_m", wheel_separation_m)) {
    ROS_ERROR_STREAM("[mower_comms_sim] Wheel separation must be specified for odometry. Quitting.");
    return 1;
  }

  if (!paramNh.getParam("mower_esc_enabled", mower_esc_enabled)) {
    ROS_ERROR_STREAM("[mower_comms_sim] Mower ESC enabled parameter is not specified. Quitting.");
    return 1;
  }

  if (paramNh.param("cmd_vel_timout", cmd_vel_timout, 1.0)) {
    ROS_INFO_STREAM("[mower_comms_sim] Configured cmd_vel_timout: " << cmd_vel_timout);
  }

  if (paramNh.param("publish_mag", publish_mag, false)) {
    ROS_INFO_STREAM("[mower_comms_sim] Configured publish_mag: " << publish_mag);
  }

  ROS_INFO_STREAM("[mower_comms_sim] Wheel radius [m]: " << wheel_radius_m << " , separation [m]: " << wheel_separation_m);

  last_cmd_twist.linear.x = 0;
  last_cmd_twist.angular.z = 0;
  speed_mow = 0;

  status_pub = n.advertise<mower_msgs::Status>("mower/status", 1);
  uss_pub = n.advertise<sensor_msgs::Range>("mower/uss", 5);
  contact_pub = n.advertise<contact_sensor_layer::Contact>("mower/bumper", 4);

  positioningClient = n.serviceClient<xbot_positioning::SetPoseSrv>("xbot_positioning/set_robot_pose");

  sensor_imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
  cmd_vel_safe_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_safe", 1);
  measured_vel_pub = n.advertise<geometry_msgs::TwistStamped>("mower/measured_vel", 1);

  ros::ServiceServer mow_service = n.advertiseService("mower_service/mow_enabled", setMowEnabled);
  ros::ServiceServer emergency_service = n.advertiseService("mower_service/emergency", setEmergencyMode);
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 0, onCmdVelReceived, ros::TransportHints().tcpNoDelay(true));
  ros::Subscriber high_level_status_sub = n.subscribe("/mower_logic/current_state", 0, highLevelStatusReceived);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.02), publishActuatorsTimerTask);

  double x,y,z,yaw;
  if (!paramNh.getParam("initial_pose_x", x)  ||
    !paramNh.getParam("initial_pose_y", y) ||
    !paramNh.getParam("initial_pose_z", z) ||
    !paramNh.getParam("initial_pose_yaw", yaw) ) {
    ROS_INFO_STREAM("[mower_comms_sim] Initial pose must be specified");
  }
  setPose(x,y,z,yaw);

  // don't change, we need to wait for arduino to boot before actually sending stuff
  ros::Rate r(10.0);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while (ros::ok()) {
      handleLowLevelIMU();
      r.sleep();
  }

  spinner.stop();

  last_cmd_twist.linear.x = 0;
  last_cmd_twist.angular.z = 0;
  publishActuators();

  return 0;
}
