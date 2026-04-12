#pragma once
#include "DriveInterface.hpp"
#include "xesc_msgs/XescState.h"
#include <hoverboard_driver/HoverboardStateStamped.h>
#include <ros/ros.h>
#include <map>
#include <string>

class HoverboardDrive : public DriveInterface {
public:
    bool init(ros::NodeHandle& nh) override {

  //ros::Subscriber rear_state_sub =
  //    n.subscribe("/rear/hoverboard_driver/state", 0, onRearStateReceived, ros::TransportHints().tcpNoDelay(true));
  //ros::Subscriber front_state_sub =
  //    n.subscribe("/front/hoverboard_driver/state", 0, onFrontStateReceived, ros::TransportHints().tcpNoDelay(true));

        front_sub_ = nh.subscribe(
            "/front/hoverboard_driver/state", 1,
            &HoverboardDrive::onFrontState, this,
            ros::TransportHints().tcpNoDelay(true));
        rear_sub_ = nh.subscribe(
            "/rear/hoverboard_driver/state", 1,
            &HoverboardDrive::onRearState, this,
            ros::TransportHints().tcpNoDelay(true));
        ROS_INFO("[HoverboardDrive] Initialized");
        return true;
    }

    void getESCStatus(WheelId wheel, bool esc_power,
                      const ros::Time& esc_enabled_time,
                      mower_msgs::ESCStatus& status) override {
        const auto& msg = getMsg(wheel);
        bool is_left = isLeft(wheel);

        // Always populate sensor fields
        status.tacho             = is_left ? msg.state.wheelL_cnt  : msg.state.wheelR_cnt;
        status.current           = is_left ? msg.state.currL_meas  : msg.state.currR_meas;
        status.temperature_motor = is_left ? msg.state.motorL_temp : msg.state.motorR_temp;
        status.temperature_pcb   = msg.state.boardTemp;
        status.rpm               = 0;

        status.xesc_status = 0;

        if (msg.state.status & hoverboard_driver::HoverboardState::STATUS_PCB_TEMP_WARN)
            status.xesc_status |= xesc_msgs::XescState::XESC_FAULT_TEMP_WARNING_PCB;
        if (msg.state.status & hoverboard_driver::HoverboardState::STATUS_PCB_TEMP_ERR)
            status.xesc_status |= xesc_msgs::XescState::XESC_FAULT_OVERTEMP_PCB;
        if (msg.state.status & hoverboard_driver::HoverboardState::STATUS_LEFT_MOTOR_TEMP_ERR && is_left)
            status.xesc_status |= xesc_msgs::XescState::XESC_FAULT_OVERTEMP_MOTOR;
        if (msg.state.status & hoverboard_driver::HoverboardState::STATUS_RIGHT_MOTOR_TEMP_ERR && !is_left)
            status.xesc_status |= xesc_msgs::XescState::XESC_FAULT_OVERTEMP_MOTOR;
        if (msg.state.status & hoverboard_driver::HoverboardState::STATUS_LEFT_MOTOR_ERR && is_left)
            status.xesc_status |= xesc_msgs::XescState::XESC_FAULT_INTERNAL_ERROR;
        if (msg.state.status & hoverboard_driver::HoverboardState::STATUS_RIGHT_MOTOR_ERR && !is_left)
            status.xesc_status |= xesc_msgs::XescState::XESC_FAULT_INTERNAL_ERROR;

        // --- Power / startup guard ---
        if (!esc_power ||
            (ros::Time::now() - esc_enabled_time).toSec() < 3.0) {
            status.status = mower_msgs::ESCStatus::ESC_STATUS_OFF;
            return;
        }

        // --- Connection check ---
        if (msg.state.connection_state !=
            hoverboard_driver::HoverboardState::HOVERBOARD_CONNECTION_STATE_CONNECTED) {
            ROS_ERROR_STREAM_THROTTLE(1,
                "[HoverboardDrive] " << wheelName(wheel)
                << " disconnected, connection_state="
                << (int)msg.state.connection_state);
            status.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
            return;
        }

        // --- Error check ---
        // Strip temperature and battery bits before checking for driver errors
        uint16_t errors = msg.state.status &
            ~(hoverboard_driver::HoverboardState::STATUS_PCB_TEMP_WARN        |
              hoverboard_driver::HoverboardState::STATUS_PCB_TEMP_ERR         |
              hoverboard_driver::HoverboardState::STATUS_LEFT_MOTOR_TEMP_ERR  |
              hoverboard_driver::HoverboardState::STATUS_RIGHT_MOTOR_TEMP_ERR |
              hoverboard_driver::HoverboardState::STATUS_BATTERY_L1           |
              hoverboard_driver::HoverboardState::STATUS_BATTERY_L2);

        if (errors) {
            ROS_ERROR_STREAM_THROTTLE(1,
                "[HoverboardDrive] " << wheelName(wheel)
                << " error, status=0x" << std::hex << msg.state.status);
            status.status = mower_msgs::ESCStatus::ESC_STATUS_ERROR;
            return;
        }

        // --- OK — check temperatures ---
        status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;

        if (msg.state.status &
            hoverboard_driver::HoverboardState::STATUS_PCB_TEMP_WARN) {
            ROS_WARN_STREAM_THROTTLE(10,
                "[HoverboardDrive] " << wheelName(wheel)
                << " PCB temperature warning: " << msg.state.boardTemp << "C");
        }
        if (msg.state.status &
            hoverboard_driver::HoverboardState::STATUS_PCB_TEMP_ERR) {
            ROS_ERROR_STREAM_THROTTLE(1,
                "[HoverboardDrive] " << wheelName(wheel)
                << " PCB overtemperature: " << msg.state.boardTemp << "C");
            status.status = mower_msgs::ESCStatus::ESC_STATUS_OVERHEATED;
            return;
        }
        if (is_left &&
            (msg.state.status &
             hoverboard_driver::HoverboardState::STATUS_LEFT_MOTOR_TEMP_ERR)) {
            ROS_WARN_STREAM_THROTTLE(10,
                "[HoverboardDrive] Left motor temperature error: "
                << msg.state.motorL_temp << "C");
            //status.status = mower_msgs::ESCStatus::ESC_STATUS_OVERHEATED;

        }
        if (!is_left &&
            (msg.state.status &
             hoverboard_driver::HoverboardState::STATUS_RIGHT_MOTOR_TEMP_ERR)) {
            ROS_WARN_STREAM_THROTTLE(10,
                "[HoverboardDrive] Right motor temperature error: "
                << msg.state.motorR_temp << "C");
            //status.status = mower_msgs::ESCStatus::ESC_STATUS_OVERHEATED;
        }
        if (msg.state.status & hoverboard_driver::HoverboardState::STATUS_BATTERY_L1 ||
            msg.state.status & hoverboard_driver::HoverboardState::STATUS_BATTERY_L2) {
            ROS_WARN_STREAM_THROTTLE(30,
                "[HoverboardDrive] " << wheelName(wheel) << " battery warning");
        }
    }

/*void convertHoverboardStatus(bool esc_power, const ros::Time& esc_enabled_time,hoverboard_driver::HoverboardStateStamped &state_msg,
                             mower_msgs::ESCStatus &ros_esc_left_status, mower_msgs::ESCStatus &ros_esc_right_status) {
  uint8_t statusNoTemperaturesNoBattery =
      state_msg.state.status &
      ~(hoverboard_driver::HoverboardState::STATUS_PCB_TEMP_WARN |
        hoverboard_driver::HoverboardState::STATUS_PCB_TEMP_ERR |
        hoverboard_driver::HoverboardState::STATUS_LEFT_MOTOR_TEMP_ERR |
        hoverboard_driver::HoverboardState::STATUS_RIGHT_MOTOR_TEMP_ERR |
        hoverboard_driver::HoverboardState::STATUS_BATTERY_L1 | hoverboard_driver::HoverboardState::STATUS_BATTERY_L2);

  if (!esc_power || (ros::Time::now() - esc_enabled_time).toSec() < 3.0) {
    // report esc off status when disabled or started less than 2 seconds ago to prevent report disconnected status
    ros_esc_left_status.status = mower_msgs::ESCStatus::ESC_STATUS_OFF;
    ros_esc_right_status.status = mower_msgs::ESCStatus::ESC_STATUS_OFF;
  } else if (state_msg.state.connection_state !=
             hoverboard_driver::HoverboardState::HOVERBOARD_CONNECTION_STATE_CONNECTED
             //&& vesc_status.state.connection_state !=
             //xesc_msgs::XescState::XESC_CONNECTION_STATE_CONNECTED_INCOMPATIBLE_FW
  ) {
    // ESC is disconnected, the bad status that never should happen
    ROS_ERROR_STREAM_THROTTLE(
        1, "[mower_comms] Hoverborad connection status: " << (int)state_msg.state.connection_state
                                                          << " status code: " << state_msg.state.status);
    ros_esc_left_status.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
    ros_esc_right_status.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
  } else if (statusNoTemperaturesNoBattery) {
    ROS_ERROR_STREAM_THROTTLE(1, "[mower_comms] Hoverborad controller status code: " << state_msg.state.status);
    // ESC has a fault
    ros_esc_left_status.status = mower_msgs::ESCStatus::ESC_STATUS_ERROR;
    ros_esc_right_status.status = mower_msgs::ESCStatus::ESC_STATUS_ERROR;
    // FIXME!!! temporary disable hoverboard errors
    // ros_esc_left_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
    // ros_esc_right_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
  } else {
    // ESC is OK but we will check temperatures
    ros_esc_left_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
    ros_esc_right_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;

    // If evrything looks ok at the moment, check temperatures
    if (state_msg.state.status & hoverboard_driver::HoverboardState::STATUS_PCB_TEMP_WARN) {
      ROS_WARN_STREAM_THROTTLE(10, "[mower_comms] Motor controller PCB temerature warning");
    }
    if (state_msg.state.status & hoverboard_driver::HoverboardState::STATUS_PCB_TEMP_ERR) {
      ros_esc_left_status.status = mower_msgs::ESCStatus::ESC_STATUS_OVERHEATED;
      ros_esc_right_status.status = mower_msgs::ESCStatus::ESC_STATUS_OVERHEATED;
    }
    if (state_msg.state.status & hoverboard_driver::HoverboardState::STATUS_LEFT_MOTOR_TEMP_ERR) {
      ROS_WARN_STREAM_THROTTLE(10, "[mower_comms] Left Motor temerature error: " << state_msg.state.motorL_temp);
      // ros_esc_left_status.status = mower_msgs::ESCStatus::ESC_STATUS_OVERHEATED;
    }
    if (state_msg.state.status & hoverboard_driver::HoverboardState::STATUS_RIGHT_MOTOR_TEMP_ERR) {
      ROS_WARN_STREAM_THROTTLE(10, "[mower_comms] Right Motor temerature error: " << state_msg.state.motorR_temp);
      // ros_esc_right_status.status = mower_msgs::ESCStatus::ESC_STATUS_OVERHEATED;
    }

    // and log battery warings
    if (state_msg.state.status & hoverboard_driver::HoverboardState::STATUS_BATTERY_L1 ||
        state_msg.state.status & hoverboard_driver::HoverboardState::STATUS_BATTERY_L2) {
      ROS_WARN_STREAM("[mower_comms] Motor controller reports battery warning");
    }
  }

  // if (abs(state_msg.state.cmdL - state_msg.state.speedL_meas) > state_msg.state.cmdL * 0.2 ) {
  //     ROS_WARN_STREAM_THROTTLE(1, "[mower_comms] Motor L stall/overrun detected cmd=" << state_msg.state.cmdL << "
  //     spd=" << state_msg.state.speedL_meas);
  // }
  // if (abs(state_msg.state.cmdR - state_msg.state.speedR_meas) > state_msg.state.cmdR * 0.2 ) {
  //     ROS_WARN_STREAM_THROTTLE(1, "[mower_comms] Motor R stall/overrun detected cmd=" << state_msg.state.cmdR << "
  //     spd=" << state_msg.state.speedR_meas);
  // }

  // TODO check .tacho compatibility with howerboard .wheelX_cnt
  ros_esc_left_status.tacho = state_msg.state.wheelL_cnt;
  ros_esc_left_status.current = state_msg.state.currL_meas;
  ros_esc_left_status.temperature_motor = state_msg.state.motorL_temp;
  ros_esc_left_status.temperature_pcb = state_msg.state.boardTemp;

  ros_esc_right_status.tacho = state_msg.state.wheelR_cnt;
  ros_esc_right_status.current = state_msg.state.currR_meas;
  ros_esc_right_status.temperature_motor = state_msg.state.motorR_temp;
  ros_esc_right_status.temperature_pcb = state_msg.state.boardTemp;
}
  
void sendWheelTickAndMeasuredTwist(ros::Time& stamp) {

  double dt = (stamp - prev_wheel_pos_stamp).toSec();
  double rl_delta_rad = last_rear_status.state.wheelL_cnt - prev_wheel_pos_rl;
  double fl_delta_rad = last_front_status.state.wheelL_cnt - prev_wheel_pos_fl;
  double rr_delta_rad = last_rear_status.state.wheelR_cnt - prev_wheel_pos_rr;
  double fr_delta_rad = last_front_status.state.wheelR_cnt - prev_wheel_pos_fr;

  prev_wheel_pos_stamp = stamp;
  prev_wheel_pos_rl = last_rear_status.state.wheelL_cnt;
  prev_wheel_pos_fl = last_front_status.state.wheelL_cnt;
  prev_wheel_pos_rr = last_rear_status.state.wheelR_cnt;
  prev_wheel_pos_fr = last_front_status.state.wheelR_cnt;


  //prev_wheel_tick_msg = wheel_tick_msg;
  if (!has_prev_wheel_tick_msg) {
    has_prev_wheel_tick_msg = true;
    return;
  }


  double d_wheel_l_rad = (rl_delta_rad + fl_delta_rad) / 2;
  double d_wheel_r_rad = (rr_delta_rad + fr_delta_rad) / 2;

  double d_wheel_l = d_wheel_l_rad * wheel_radius_m;
  double d_wheel_r = d_wheel_r_rad * wheel_radius_m;

  double d_linear = (d_wheel_r + d_wheel_l) / 2.0;
  double d_angular = (d_wheel_r - d_wheel_l) / wheel_separation_m;
  geometry_msgs::TwistStamped measured_wheel_tick_twist;
  measured_wheel_tick_twist.header.frame_id = "base_link";
  measured_wheel_tick_twist.header.stamp = stamp;
  measured_wheel_tick_twist.twist.linear.x = d_linear/dt;
  measured_wheel_tick_twist.twist.angular.z = d_angular/dt;

  measured_vel_pub.publish(measured_wheel_tick_twist);

  #ifdef HOVERBOARD_ODOM
    geometry_msgs::TwistStamped measured_odom_twist;
    measured_odom_twist.header.frame_id = "base_link";
    measured_odom_twist.header.stamp = stamp;
    measured_odom_twist.twist.linear.x = (last_rear_odom.twist.twist.linear.x + last_front_odom.twist.twist.linear.x)/2;
    measured_odom_twist.twist.angular.z = (last_rear_odom.twist.twist.angular.z + last_front_odom.twist.twist.angular.z)/2;
    //measured_vel_pub.publish(measured_odom_twist);
  #endif

  //ROS_INFO("[mower_comms] WheelTickTwist %+5.3f %+5.3f OdomTwist %+5.3f %+5.3f",
  //  measured_wheel_tick_twist.twist.linear.x,measured_wheel_tick_twist.twist.angular.z,
  //  measured_odom_twist.twist.linear.x,measured_odom_twist.twist.angular.z);
}

*/

    double getWheelPosition(WheelId wheel) override {
        const auto& msg = getMsg(wheel);
        return isLeft(wheel) ? msg.state.wheelL_cnt : msg.state.wheelR_cnt;
    }

    uint8_t getAxleStatusAge(WheelId left, WheelId right) override {
        const auto& msg = isFront(left) ? front_msg_ : rear_msg_;
        if (msg.state.connection_state ==
            hoverboard_driver::HoverboardState::HOVERBOARD_CONNECTION_STATE_DISCONNECTED) {
            return UINT8_MAX;
        }
        double age = (ros::Time::now() - msg.header.stamp).toSec();
        return age > UINT8_MAX ? UINT8_MAX : static_cast<uint8_t>(age);
    }
private:
    hoverboard_driver::HoverboardStateStamped front_msg_;
    hoverboard_driver::HoverboardStateStamped rear_msg_;
    ros::Subscriber front_sub_;
    ros::Subscriber rear_sub_;

    void onFrontState(const hoverboard_driver::HoverboardStateStamped::ConstPtr& msg) {
        front_msg_ = *msg;
    }
    void onRearState(const hoverboard_driver::HoverboardStateStamped::ConstPtr& msg) {
        rear_msg_ = *msg;
    }

    const hoverboard_driver::HoverboardStateStamped& getMsg(WheelId wheel) const {
        return isFront(wheel) ? front_msg_ : rear_msg_;
    }
};