#pragma once
#include "DriveInterface.hpp"
#include "xesc_msgs/XescState.h"
#include <odrive_can/ControllerStatus.h>
#include <odrive_can/ODriveStatus.h>
#include <ros/ros.h>
#include <map>
#include <string>

// ODrive active_errors bitmask values from odrive_enums.h
// Listed here for reference when mapping to xesc_status
// ODRIVE_ERROR_INITIALIZING               = 0x01
// ODRIVE_ERROR_SYSTEM_LEVEL               = 0x02
// ODRIVE_ERROR_TIMING_ERROR               = 0x04
// ODRIVE_ERROR_MISSING_ESTIMATE           = 0x08
// ODRIVE_ERROR_BAD_CONFIG                 = 0x10
// ODRIVE_ERROR_DRV_FAULT                  = 0x20
// ODRIVE_ERROR_MISSING_INPUT              = 0x40
// ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE        = 0x100
// ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE       = 0x200
// ODRIVE_ERROR_DC_BUS_OVER_CURRENT        = 0x400
// ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT  = 0x800
// ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION    = 0x1000
// ODRIVE_ERROR_MOTOR_OVER_TEMP            = 0x2000
// ODRIVE_ERROR_INVERTER_OVER_TEMP         = 0x4000
// ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION   = 0x8000
// ODRIVE_ERROR_POSITION_LIMIT_VIOLATION   = 0x10000
// ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED     = 0x1000000
// ODRIVE_ERROR_ESTOP_REQUESTED            = 0x2000000
// ODRIVE_ERROR_SPINOUT_DETECTED           = 0x4000000
// ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED    = 0x8000000
// ODRIVE_ERROR_THERMISTOR_DISCONNECTED    = 0x10000000

static constexpr float FET_TEMP_WARN_C    = 70.0f;
static constexpr float FET_TEMP_ERROR_C   = 80.0f;
static constexpr float MOTOR_TEMP_WARN_C  = 80.0f;
static constexpr float MOTOR_TEMP_ERROR_C = 100.0f;

class ODriveDrive : public DriveInterface {
public:
    bool init(ros::NodeHandle& nh) override {
        for (const auto& name : {"front_left_wheel", "front_right_wheel",
                              "rear_left_wheel",  "rear_right_wheel"}) {
            ctrl_status_[name] = odrive_can::ControllerStatus();
            odrv_status_[name] = odrive_can::ODriveStatus();
        }

        ctrl_sub_ = nh.subscribe(
            "controller_status", 10,
            &ODriveDrive::onCtrlStatus, this,
            ros::TransportHints().tcpNoDelay(true));
        odrv_sub_ = nh.subscribe(
            "odrive_status", 10,
            &ODriveDrive::onOdrvStatus, this,
            ros::TransportHints().tcpNoDelay(true));
        
        ROS_INFO("[ODriveDrive] Initialized, subscribed to controller_status and odrive_status");
        return true;
    }

    void getESCStatus(WheelId wheel, bool esc_power,
                      const ros::Time& esc_enabled_time,
                      mower_msgs::ESCStatus& status) override {
        const auto& ctrl = getCtrl(wheel);
        const auto& odrv = getOdrv(wheel);

        // Always populate sensor fields
        status.tacho             = ctrl.pos_estimate;
        status.current           = ctrl.iq_measured;
        status.temperature_motor = odrv.motor_temperature;
        status.temperature_pcb   = odrv.fet_temperature;
        status.rpm               = 0; // not provided by ODrive CAN
        status.xesc_status       = mapToXescStatus(ctrl, odrv);

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
            status.xesc_status |= xesc_msgs::XescState::XESC_FAULT_WATCHDOG;
            return;
        }

        // --- Error check ---
        if (ctrl.active_errors) {
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
        }
        if (odrv.fet_temperature > FET_TEMP_ERROR_C) {
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
        }
        if (odrv.motor_temperature > MOTOR_TEMP_ERROR_C) {
            ROS_ERROR_STREAM_THROTTLE(1,
                "[ODriveDrive] " << wheelName(wheel)
                << " motor overtemperature: " << odrv.motor_temperature << "C");
            status.status = mower_msgs::ESCStatus::ESC_STATUS_OVERHEATED;
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
        ctrl_status_[msg->header.frame_id] = *msg;
    }
    void onOdrvStatus(const odrive_can::ODriveStatus::ConstPtr& msg) {
        odrv_status_[msg->header.frame_id] = *msg;
    }

    const odrive_can::ControllerStatus& getCtrl(WheelId wheel) const {
        return ctrl_status_.at(wheelName(wheel));
    }
    const odrive_can::ODriveStatus& getOdrv(WheelId wheel) const {
        return odrv_status_.at(wheelName(wheel));
    }

    // Map ODrive active_errors bitmask to xesc_status bitmask
    static uint32_t mapToXescStatus(
        const odrive_can::ControllerStatus& ctrl,
        const odrive_can::ODriveStatus& odrv)
    {
        uint32_t xesc = 0;
        uint32_t err  = ctrl.active_errors;

        if (err & 0x000001) xesc |= xesc_msgs::XescState::XESC_FAULT_UNINITIALIZED;
        if (err & 0x000004) xesc |= xesc_msgs::XescState::XESC_FAULT_WATCHDOG;       // TIMING_ERROR
        if (err & 0x000008) xesc |= xesc_msgs::XescState::XESC_FAULT_INVALID_HALL;   // MISSING_ESTIMATE
        if (err & 0x000002 ||
            err & 0x000010 ||
            err & 0x000020 ||
            err & 0x000040) xesc |= xesc_msgs::XescState::XESC_FAULT_INTERNAL_ERROR; // SYSTEM/BAD_CONFIG/DRV/MISSING_INPUT
        if (err & 0x000100) xesc |= xesc_msgs::XescState::XESC_FAULT_OVERVOLTAGE;    // DC_BUS_OVER_VOLTAGE
        if (err & 0x000200) xesc |= xesc_msgs::XescState::XESC_FAULT_UNDERVOLTAGE;   // DC_BUS_UNDER_VOLTAGE
        if (err & 0x000400 ||
            err & 0x000800 ||
            err & 0x001000) xesc |= xesc_msgs::XescState::XESC_FAULT_OVERCURRENT;    // OVER_CURRENT variants
        if (err & 0x002000) xesc |= xesc_msgs::XescState::XESC_FAULT_OVERTEMP_MOTOR; // MOTOR_OVER_TEMP
        if (err & 0x004000) xesc |= xesc_msgs::XescState::XESC_FAULT_OVERTEMP_PCB;   // INVERTER_OVER_TEMP
        if (err & 0x1000000) xesc |= xesc_msgs::XescState::XESC_FAULT_WATCHDOG;      // WATCHDOG_TIMER_EXPIRED
        if (err & 0x10000000) xesc |= xesc_msgs::XescState::XESC_FAULT_OPEN_LOAD;    // THERMISTOR_DISCONNECTED

        // Temperature warnings from ODriveStatus
        if (odrv.fet_temperature   > FET_TEMP_WARN_C)
            xesc |= xesc_msgs::XescState::XESC_FAULT_TEMP_WARNING_PCB;

        return xesc;
    }
};