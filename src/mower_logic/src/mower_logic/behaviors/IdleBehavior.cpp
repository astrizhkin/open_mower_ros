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
#include "IdleBehavior.h"

#include "PerimeterDocking.h"

extern void stopMoving(std::string reason);
extern void setEmergencyMode(bool emergency);
extern void setGPS(bool enabled, std::string reason);
extern void setRobotPose(geometry_msgs::Pose &pose, std::string reason);
extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo> &actions);
extern ros::Time rain_resume;

extern ros::ServiceClient dockingPointClient;
extern mower_msgs::Status getStatus();
extern mower_logic::MowerLogicConfig getConfig();
extern dynamic_reconfigure::Server<mower_logic::MowerLogicConfig> *reconfigServer;

extern ros::ServiceClient getAreasClient;
extern ros::ServiceClient dockingPointClient;

IdleBehavior IdleBehavior::INSTANCE(false);
IdleBehavior IdleBehavior::DOCKED_INSTANCE(true);

std::string IdleBehavior::state_name() {
  return "IDLE";
}

Behavior *IdleBehavior::execute() {
  // Check, if we have a configured map. If not, print info and go to area recorder
  mower_map::GetMowingAreasSrv mapSrv;
  if (!getAreasClient.call(mapSrv)) {
    ROS_WARN("[IdleBehavior] We don't have a mowing areas configured. Starting Area Recorder!");
    return &AreaRecordingBehavior::INSTANCE;
  }

  // Check, if we have a docking position. If not, print info and go to area recorder
  mower_map::GetDockingPointSrv get_docking_point_srv;
  if (!dockingPointClient.call(get_docking_point_srv)) {
    ROS_WARN("[IdleBehavior] We don't have a docking point configured. Starting Area Recorder!");
    return &AreaRecordingBehavior::INSTANCE;
  }

  setGPS(false, "idle");
  geometry_msgs::PoseStamped docking_pose_stamped;
  docking_pose_stamped.pose = get_docking_point_srv.response.docking_pose;
  docking_pose_stamped.header.frame_id = "map";
  docking_pose_stamped.header.stamp = ros::Time::now();

  ros::Rate r(25);
  while (ros::ok()) {
    setMowerEnabled(false);
    stopMoving("idle");
    const auto last_config = getConfig();
    const auto last_status = getStatus();

    const bool automatic_mode = last_config.automatic_mode == eAutoMode::AUTO;
    const bool active_semiautomatic_task = last_config.automatic_mode == eAutoMode::SEMIAUTO &&
                                           shared_state->active_semiautomatic_task &&
                                           !shared_state->semiautomatic_task_paused;
    const bool rain_delay = last_config.rain_mode == 2 && ros::Time::now() < rain_resume;
    if (rain_delay) {
      ROS_INFO_STREAM_THROTTLE(300, "[IdleBehavior] Rain delay: " << int((rain_resume - ros::Time::now()).toSec() / 60) << " minutes");
    }
    const bool mower_ready =
        (last_config.charge_start_soc == 0 || last_status.battery_soc > last_config.charge_start_soc) &&
        (last_config.charge_start_voltage == 0 || last_status.v_battery > last_config.charge_start_voltage) &&

        last_status.mow_esc_status.temperature_motor < last_config.motor_cold_temperature &&
        !last_config.manual_pause_mowing && !rain_delay;

    if (manual_start_mowing || ((automatic_mode || active_semiautomatic_task) && mower_ready)) {
      if(manual_start_mowing) {
        MowingBehavior::INSTANCE.setMowingAreas(manual_mowing_areas);
      }

      // set the robot's position to the dock if we're actually docked
      if (last_status.v_charge > last_config.charger_min_voltage) {
        ROS_INFO_STREAM("[IdleBehavior] Currently inside the docking station, we set the robot's pose to the docks pose.");
        setRobotPose(docking_pose_stamped.pose, "idle prepare for undocking");
        if (PerimeterUndockingBehavior::configured(config)) {
          return &PerimeterUndockingBehavior::INSTANCE;
        }
        return &UndockingBehavior::INSTANCE;
      }
      // Not docked, so just mow
      ROS_INFO_STREAM("[IdleBehavior] Currently undocked, just start mowing.");
      setGPS(true, "idle prepare for mowing");
      return &MowingBehavior::INSTANCE;
    }

    if (start_area_recorder) {
      return &AreaRecordingBehavior::INSTANCE;
    }

    if (start_debug) {
      return &DebugBehavior::INSTANCE;
    }

    // This gets called if we need to refresh, e.g. on clearing maps
    if (aborted) {
      return &IdleBehavior::INSTANCE;
    }

    if (last_config.docking_redock && stay_docked && last_status.v_charge < 5.0) {
      ROS_WARN("We docked but seem to have lost contact with the charger.  Undocking and trying again!");
      return &UndockingBehavior::RETRY_INSTANCE;
    }

    r.sleep();
  }

  return nullptr;
}

void IdleBehavior::enter() {
  mower_enabled_flag = mower_enabled_flag_before_pause = false;
  start_debug = false;
  start_area_recorder = false;
  // Reset the docking behavior, to allow docking
  DockingBehavior::INSTANCE.reset();

  // disable it, so that we don't start mowing immediately
  manual_start_mowing = false;

  for (auto &a : actions) {
    a.enabled = true;
  }
  registerActions("mower_logic:idle", actions);
}

void IdleBehavior::exit() {
  for (auto &a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:idle", actions);
}

void IdleBehavior::reset() {
}

bool IdleBehavior::needs_gps() {
  return false;
}

void IdleBehavior::command_start() {
  ROS_INFO_STREAM("[IdleBehavior] Got start_mowing command");
  if(shared_state == nullptr) {
    ROS_ERROR_STREAM("[IdleBehavior] Logic shared state is not initialized. Abort start action.");
    return;
  }

  // We got start, so we can reset the last manual pause
  shared_state->semiautomatic_task_paused = false;
  manual_start_mowing = true;
  manual_mowing_areas = MowingBehavior::ALL_AREAS;
}

void IdleBehavior::command_s1() {
  ROS_INFO_STREAM("[IdleBehavior] Got start_area_recording command");
  start_area_recorder = true;
}

void IdleBehavior::command_s2() {
  ROS_INFO_STREAM("[IdleBehavior] Got start_debug command");
  start_debug = true;
}

bool IdleBehavior::redirect_joystick() {
  return false;
}

uint8_t IdleBehavior::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_IDLE;
}

IdleBehavior::IdleBehavior(bool stayDocked) {
  this->stay_docked = stayDocked;
  this->sub_state = 0;

  actions.clear();

  actions.push_back(createAction("start_mowing","Start Mowing"));
  actions.push_back(createAction("start_area_recording","Start Area Recording"));
  actions.push_back(createAction("start_debug","Start Debug"));
}

void IdleBehavior::handle_action(const std::string& action, const std::string& parameters) {
  if (action == "mower_logic:idle/start_mowing") {
    command_start();
    if(!parameters.empty()){
        manual_mowing_areas = parameters;
    }
  } else if (action == "mower_logic:idle/start_area_recording") {
    command_s1();
  } else if (action == "mower_logic:idle/start_debug") {
    command_s2();
  }
}
