
#ifndef SRC_MANUAL_MODE_BEHAVIOR_H
#define SRC_MANUAL_MODE_BEHAVIOR_H

#include "Behavior.h"
#include "IdleBehavior.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

class ManualBehavior : public Behavior {
 public:
  static ManualBehavior INSTANCE;

  ManualBehavior();

 private:
  sensor_msgs::Joy last_joy;

  ros::Subscriber joy_sub;

 private:
  void joy_received(const sensor_msgs::Joy &joy_msg);
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

  bool redirect_joystick() override;

  uint8_t get_state() override;

  void handle_action(const std::string& action, const std::string& parameters = std::string()) override;
};

#endif  // SRC_MANUAL_MODE_BEHAVIOR_H
