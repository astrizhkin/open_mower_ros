#pragma once
#include <ros/ros.h>
#include <mower_msgs/ESCStatus.h>
#include <geometry_msgs/Twist.h>

enum class WheelId {
    FRONT_LEFT,
    FRONT_RIGHT,
    REAR_LEFT,
    REAR_RIGHT
};

class DriveInterface {
public:
    virtual ~DriveInterface() = default;

    virtual bool init(ros::NodeHandle& nh) = 0;

    // Fill all ESCStatus fields including status code
    virtual void getESCStatus(WheelId wheel, bool esc_power,
                              const ros::Time& esc_enabled_time,
                              mower_msgs::ESCStatus& status) = 0;

    // Wheel position in radians for odometry delta
    virtual double getWheelPosition(WheelId wheel) = 0;

    virtual uint8_t getAxleStatusAge(WheelId left, WheelId right) = 0;

    // Shared wheel helpers available to all implementations and callers
    static bool isLeft(WheelId wheel) {
        return wheel == WheelId::FRONT_LEFT || wheel == WheelId::REAR_LEFT;
    }

    static bool isFront(WheelId wheel) {
        return wheel == WheelId::FRONT_LEFT || wheel == WheelId::FRONT_RIGHT;
    }

    static const std::string& wheelName(WheelId wheel) {
        static const std::map<WheelId, std::string> names = {
            {WheelId::FRONT_LEFT,  "front_left"},
            {WheelId::FRONT_RIGHT, "front_right"},
            {WheelId::REAR_LEFT,   "rear_left"},
            {WheelId::REAR_RIGHT,  "rear_right"},
        };
        return names.at(wheel);
    }
};