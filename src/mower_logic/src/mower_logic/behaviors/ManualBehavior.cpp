#include "ManualBehavior.h"

extern ros::NodeHandle *n;
extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo> &actions);

extern void stop();

extern bool setGPS(bool enabled, std::string reason);

ManualBehavior ManualBehavior::INSTANCE;

std::string ManualBehavior::state_name() {
  return "MANUAL";
}

Behavior *ManualBehavior::execute() {
  setGPS(true, "manual mode");
  sub_state = 0;

  ros::Rate inputDelay(ros::Duration().fromSec(0.1));
  while (ros::ok() && !aborted) {
    inputDelay.sleep();
  }

  return &IdleBehavior::INSTANCE;
}

void ManualBehavior::enter() {
  update_actions();

  mower_enabled_flag = mower_enabled_flag_before_pause = paused = aborted = false;
  //default enable mower for manual mowing
  mower_enabled_flag = true;

  ROS_INFO_STREAM("[ManualBehavior] Starting manual mode");
  ROS_INFO_STREAM("[ManualBehavior] Subscribing to /joy for user input");
  joy_sub = n->subscribe("/joy", 100, &ManualBehavior::joy_received, this);
}

void ManualBehavior::exit() {
  for (auto &a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:manual_mode", actions);

  joy_sub.shutdown();
}

void ManualBehavior::reset() {
}

bool ManualBehavior::needs_gps() {
  return false;
}

void ManualBehavior::joy_received(const sensor_msgs::Joy &joy_msg) {
  last_joy = joy_msg;
}


void ManualBehavior::command_home() {
  abort();
}

bool ManualBehavior::redirect_joystick() {
  return true;
}

uint8_t ManualBehavior::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_MANUAL;
}

std::string ManualBehavior::sub_state_name() {
  return "";
}

void ManualBehavior::handle_action(const std::string& action, const std::string& parameters) {
  ROS_INFO_STREAM("[ManualBehavior] Got action " << action);
  if (action == "mower_logic:manual_mode/start_manual_mowing") {
    setMowerEnabled(true);
  } else if (action == "mower_logic:manual_mode/stop_manual_mowing") {
    setMowerEnabled(false);
  } else if(action == "mower_logic:manual_mode/abort_manual") {
    this->abort();
  }
  update_actions();
}

ManualBehavior::ManualBehavior() {
  actions.clear();
  actions.push_back(createAction("start_manual_mowing","Start manual mowing"));
  actions.push_back(createAction("stop_manual_mowing","Stop manual mowing"));
  actions.push_back(createAction("abort_manual","Stop manual mowing"));
}

void ManualBehavior::update_actions() {
  disableAllActions();
  getAction("start_manual_mowing").enabled = !mower_enabled_flag;
  getAction("stop_manual_mowing").enabled = mower_enabled_flag;
  getAction("abort_manual").enabled = true;
  registerActions("mower_logic:manual_mode", actions);
}


