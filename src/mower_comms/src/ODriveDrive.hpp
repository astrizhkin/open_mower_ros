#pragma once
#include "DriveInterface.hpp"
#include "xesc_msgs/XescState.h"
#include <odrive_can/ControllerStatus.h>
#include <odrive_can/ODriveStatus.h>
#include <odrive_enums.h>
#include <ros/ros.h>
#include <map>
#include <string>

class ODriveDrive : public DriveInterface {
public:
    bool init(ros::NodeHandle& nh) override {
        ctrl_sub_ = nh.subscribe(
            "/odrive_driver/controller_status", 10,
            &ODriveDrive::onCtrlStatus, this,
            ros::TransportHints().tcpNoDelay(true));
        odrv_sub_ = nh.subscribe(
            "/odrive_driver/odrive_status", 10,
            &ODriveDrive::onOdrvStatus, this,
            ros::TransportHints().tcpNoDelay(true));
        
        ROS_INFO("[ODriveDrive] Initialized, subscribed to controller_status and odrive_status");
        return true;
    }


    static constexpr float FET_TEMP_WARN_C   = 65.0f;
    static constexpr float FET_TEMP_ERROR_C   = 75.0f;

    static constexpr float MOTOR_TEMP_WARN_C  = 70.0f;
    static constexpr float MOTOR_TEMP_ERROR_C = 80.0f;

    void getESCStatus(WheelId wheel, bool esc_power,
                      const ros::Time& esc_enabled_time,
                      mower_msgs::ESCStatus& status) override {
        const auto& ctrl = getCtrl(wheel);
        const auto& odrv = getOdrv(wheel);

        // Always populate sensor fields
        status.tacho             = ctrl.pos_estimate;
        status.current           = ctrl.iq_measured;
        status.rpm               = ctrl.vel_estimate;
        status.temperature_motor = odrv.motor_temperature;
        status.temperature_pcb   = odrv.fet_temperature;
        status.xesc_status       = mapToXescStatus(ctrl.active_errors);

        // --- Power / startup guard ---
        if (!esc_power ||
            (ros::Time::now() - esc_enabled_time).toSec() < 3.0) {
            status.status = mower_msgs::ESCStatus::ESC_STATUS_OFF;
            return;
        }

        // --- Connection check ---
        if (!ctrl.connected) {
            ROS_ERROR_STREAM_THROTTLE(1,
                "[ODriveDrive] " << wheelName(wheel) << " disconnected");
            status.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
            return;
        }

        // Strip temperature errors
        uint32_t errors = ctrl.active_errors &
            ~(ODRIVE_ERROR_INVERTER_OVER_TEMP |
              ODRIVE_ERROR_MOTOR_OVER_TEMP);

        // --- Error check ---
        if (errors) {
            ROS_ERROR_STREAM_THROTTLE(1,
                "[ODriveDrive] " << wheelName(wheel)
                << " active_errors=0x" << std::hex << ctrl.active_errors
                << " disarm_reason=0x" << std::hex << odrv.disarm_reason);
            status.status = mower_msgs::ESCStatus::ESC_STATUS_ERROR;
            return;
        }

        // --- OK — check temperatures ---
        status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;

        if (odrv.fet_temperature > FET_TEMP_WARN_C) {
            ROS_WARN_STREAM_THROTTLE(10,
                "[ODriveDrive] " << wheelName(wheel)
                << " FET temperature warning: " << odrv.fet_temperature << "C");

            // Temperature warnings from ODriveStatus (not in active_errors)
            status.xesc_status |= xesc_msgs::XescState::XESC_FAULT_TEMP_WARNING_PCB;
        }
        if ((status.xesc_status & xesc_msgs::XescState::XESC_FAULT_OVERTEMP_PCB)
             || odrv.fet_temperature > FET_TEMP_ERROR_C) {
            ROS_ERROR_STREAM_THROTTLE(1,
                "[ODriveDrive] " << wheelName(wheel)
                << " FET overtemperature: " << odrv.fet_temperature << "C");
            status.status = mower_msgs::ESCStatus::ESC_STATUS_OVERHEATED;
            return;
        }
        if (odrv.motor_temperature > MOTOR_TEMP_WARN_C) {
            ROS_WARN_STREAM_THROTTLE(10,
                "[ODriveDrive] " << wheelName(wheel)
                << " motor temperature warning: " << odrv.motor_temperature << "C");
            // Temperature warnings from ODriveStatus (not in active_errors)
            status.xesc_status |= xesc_msgs::XescState::XESC_FAULT_TEMP_WARNING_MOTOR;
        }
        if ((status.xesc_status & xesc_msgs::XescState::XESC_FAULT_OVERTEMP_MOTOR) 
            || odrv.motor_temperature > MOTOR_TEMP_ERROR_C) {
            ROS_ERROR_STREAM_THROTTLE(1,
                "[ODriveDrive] " << wheelName(wheel)
                << " motor overtemperature: " << odrv.motor_temperature << "C");
            //status.status = mower_msgs::ESCStatus::ESC_STATUS_OVERHEATED;
            return;
        }
    }

    double getWheelPosition(WheelId wheel) override {
        return getCtrl(wheel).pos_estimate;
    }

    uint8_t getAxleStatusAge(WheelId left, WheelId right) override {
        // ODrive has per-wheel stamps — use the older of the two
        ros::Time left_stamp  = getCtrl(left).header.stamp;
        ros::Time right_stamp = getCtrl(right).header.stamp;
        ros::Time older = (left_stamp < right_stamp) ? left_stamp : right_stamp;

        bool connected = getCtrl(left).connected && getCtrl(right).connected;
        if (!connected || older.isZero()) return UINT8_MAX;

        double age = (ros::Time::now() - older).toSec();
        return age > UINT8_MAX ? UINT8_MAX : static_cast<uint8_t>(age);
    }

private:
    std::map<std::string, odrive_can::ControllerStatus> ctrl_status_;
    std::map<std::string, odrive_can::ODriveStatus>     odrv_status_;
    ros::Subscriber ctrl_sub_;
    ros::Subscriber odrv_sub_;

    void onCtrlStatus(const odrive_can::ControllerStatus::ConstPtr& msg) {
        ROS_INFO_STREAM("[ODriveDrive] ControllerStatus received for '" << msg->header.frame_id << "'");
        ctrl_status_[msg->header.frame_id] = *msg;
    }
    void onOdrvStatus(const odrive_can::ODriveStatus::ConstPtr& msg) {
        ROS_INFO_STREAM("[ODriveDrive] ODriveStatus received for '" << msg->header.frame_id << "'");
        odrv_status_[msg->header.frame_id] = *msg;
    }

    const odrive_can::ControllerStatus& getCtrl(WheelId wheel) const {
        static const odrive_can::ControllerStatus empty;
        auto it = ctrl_status_.find(wheelName(wheel));
        if (it == ctrl_status_.end()) {
            ROS_WARN_STREAM_THROTTLE(5.0, "[ODriveDrive] No ControllerStatus received yet for '" << wheelName(wheel) << "'");
            return empty;
        }
        return it->second;    
    }
    const odrive_can::ODriveStatus& getOdrv(WheelId wheel) const {
        static const odrive_can::ODriveStatus empty;
        auto it = odrv_status_.find(wheelName(wheel));
        if (it == odrv_status_.end()) {
            ROS_WARN_STREAM_THROTTLE(5.0, "[ODriveDrive] No ODriveStatus received yet for '" << wheelName(wheel) << "'");
            return empty;
        }
        return it->second;
    }

    static uint32_t mapToXescStatus(uint32_t err) {
        uint32_t xesc = 0;

        if (err == ODRIVE_ERROR_NONE)
            return xesc;

        // Uninitialized
        if (err & ODRIVE_ERROR_INITIALIZING)
            xesc |= xesc_msgs::XescState::XESC_FAULT_UNINITIALIZED;

        // Watchdog / timing
        if (err & ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED)
            xesc |= xesc_msgs::XescState::XESC_FAULT_WATCHDOG;

        // Internal errors
        if (err & ODRIVE_ERROR_TIMING_ERROR)
            xesc |= xesc_msgs::XescState::XESC_FAULT_INTERNAL_ERROR;
        if (err & ODRIVE_ERROR_SYSTEM_LEVEL)
            xesc |= xesc_msgs::XescState::XESC_FAULT_INTERNAL_ERROR;
        if (err & ODRIVE_ERROR_BAD_CONFIG)
            xesc |= xesc_msgs::XescState::XESC_FAULT_INTERNAL_ERROR;
        if (err & ODRIVE_ERROR_DRV_FAULT)
            xesc |= xesc_msgs::XescState::XESC_FAULT_INTERNAL_ERROR;
        if (err & ODRIVE_ERROR_MISSING_INPUT)
            xesc |= xesc_msgs::XescState::XESC_FAULT_INTERNAL_ERROR;
        if (err & ODRIVE_ERROR_ESTOP_REQUESTED)
            xesc |= xesc_msgs::XescState::XESC_FAULT_INTERNAL_ERROR;
        if (err & ODRIVE_ERROR_CALIBRATION_ERROR)
            xesc |= xesc_msgs::XescState::XESC_FAULT_INTERNAL_ERROR;
        if (err & ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED)
            xesc |= xesc_msgs::XescState::XESC_FAULT_INTERNAL_ERROR;

        // Hall / encoder
        if (err & ODRIVE_ERROR_SPINOUT_DETECTED)
            xesc |= xesc_msgs::XescState::XESC_FAULT_INVALID_HALL;
        if (err & ODRIVE_ERROR_MISSING_ESTIMATE)
            xesc |= xesc_msgs::XescState::XESC_FAULT_INVALID_HALL;
        if (err & ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION)
            xesc |= xesc_msgs::XescState::XESC_FAULT_INVALID_HALL;
        if (err & ODRIVE_ERROR_POSITION_LIMIT_VIOLATION)
            xesc |= xesc_msgs::XescState::XESC_FAULT_INVALID_HALL;

        // Voltage
        if (err & ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE)
            xesc |= xesc_msgs::XescState::XESC_FAULT_OVERVOLTAGE;
        if (err & ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE)
            xesc |= xesc_msgs::XescState::XESC_FAULT_UNDERVOLTAGE;

        // Current
        if (err & ODRIVE_ERROR_DC_BUS_OVER_CURRENT)
            xesc |= xesc_msgs::XescState::XESC_FAULT_OVERCURRENT;
        if (err & ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT)
            xesc |= xesc_msgs::XescState::XESC_FAULT_OVERCURRENT;
        if (err & ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION)
            xesc |= xesc_msgs::XescState::XESC_FAULT_OVERCURRENT;

        // Temperature
        if (err & ODRIVE_ERROR_MOTOR_OVER_TEMP)
            xesc |= xesc_msgs::XescState::XESC_FAULT_OVERTEMP_MOTOR;
        if (err & ODRIVE_ERROR_THERMISTOR_DISCONNECTED)
            xesc |= xesc_msgs::XescState::XESC_FAULT_OVERTEMP_MOTOR;
        if (err & ODRIVE_ERROR_INVERTER_OVER_TEMP)
            xesc |= xesc_msgs::XescState::XESC_FAULT_OVERTEMP_PCB;

        return xesc;
    }
};