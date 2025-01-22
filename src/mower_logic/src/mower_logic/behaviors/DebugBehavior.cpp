// Created by Clemens Elflein on 2/21/22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
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
#include "DebugBehavior.h"

extern ros::ServiceClient dockingPointClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfClientExePath;
extern xbot_msgs::AbsolutePose getPose();
extern mower_msgs::Status getStatus();

extern void setRobotPose(geometry_msgs::Pose &pose, std::string reason);
extern void stopMoving(std::string reason);
extern bool isGpsGood();
extern bool setGPS(bool enabled, std::string reason);

extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo> &actions);

DebugBehavior DebugBehavior::INSTANCE;

std::string DebugBehavior::state_name() {
    return "DEBUG";
}

void DebugBehavior::ellipse(nav_msgs::Path &path, double hRad, double vRad) {
    xbot_msgs::AbsolutePose pose = getPose();

    int point_count = 12;

    for (int i = 0; i < point_count ; i++) {
        double angle = 2 * M_PI * i / point_count;
        while(angle > M_PI) {
            angle -= 2 * M_PI;
        }
        geometry_msgs::PoseStamped docking_pose_stamped_front;
        docking_pose_stamped_front.pose = pose.pose.pose;
        docking_pose_stamped_front.header = pose.header;
        docking_pose_stamped_front.pose.position.x += cos(angle) * hRad;
        docking_pose_stamped_front.pose.position.y += sin(angle) * vRad;
        double tangentAngle = atan2(vRad * cos(angle), -hRad * sin(angle));
        docking_pose_stamped_front.pose.orientation = tf2::toMsg(tf2::Quaternion(0,0,tangentAngle));
        path.poses.push_back(docking_pose_stamped_front);
    }

}

void DebugBehavior::circle(nav_msgs::Path &path, double radius) {
    ellipse(path, radius, radius);
}

void DebugBehavior::square(nav_msgs::Path &path, double width) {
    rectangle(path, width, width);
}

void DebugBehavior::rectangle(nav_msgs::Path &path, double width, double height) {
   xbot_msgs::AbsolutePose pose = getPose();

    double wOffset = 0;
    double hOffset = 0;


    int point_count = 12;

    //bottom left corner, move right
    for (int i = 0; i < point_count ; i++) {
        double xPos = (width * i / point_count) + wOffset;
        double yPos = 0 + hOffset;
        double tangentAngle = 0 * 2 * M_PI / 360;
        while(tangentAngle >= M_PI) {
            tangentAngle -= 2 * M_PI;
        }
        geometry_msgs::PoseStamped docking_pose_stamped_front;
        docking_pose_stamped_front.pose = pose.pose.pose;
        docking_pose_stamped_front.header = pose.header;
        docking_pose_stamped_front.pose.position.x += xPos;
        docking_pose_stamped_front.pose.position.y += yPos;
        docking_pose_stamped_front.pose.orientation = tf2::toMsg(tf2::Quaternion(0,0,tangentAngle));
        path.poses.push_back(docking_pose_stamped_front);
    }
    //bottom right corner, move up
    for (int i = 0; i < point_count ; i++) {
        double xPos = width + wOffset;
        double yPos = (height * i / point_count) + hOffset;
        double tangentAngle = 90 * 2 * M_PI / 360;
        while(tangentAngle >= M_PI) {
            tangentAngle -= 2 * M_PI;
        }
        geometry_msgs::PoseStamped docking_pose_stamped_front;
        docking_pose_stamped_front.pose = pose.pose.pose;
        docking_pose_stamped_front.header = pose.header;
        docking_pose_stamped_front.pose.position.x += xPos;
        docking_pose_stamped_front.pose.position.y += yPos;
        docking_pose_stamped_front.pose.orientation = tf2::toMsg(tf2::Quaternion(0,0,tangentAngle));
        path.poses.push_back(docking_pose_stamped_front);
    }
    //top right corner, meve left
    for (int i = 0; i < point_count ; i++) {
        double xPos = width -(width * i / point_count) + wOffset;
        double yPos = height + hOffset;
        double tangentAngle = 180 * 2 * M_PI / 360;
        while(tangentAngle >= M_PI) {
            tangentAngle -= 2 * M_PI;
        }
        geometry_msgs::PoseStamped docking_pose_stamped_front;
        docking_pose_stamped_front.pose = pose.pose.pose;
        docking_pose_stamped_front.header = pose.header;
        docking_pose_stamped_front.pose.position.x += xPos;
        docking_pose_stamped_front.pose.position.y += yPos;
        docking_pose_stamped_front.pose.orientation = tf2::toMsg(tf2::Quaternion(0,0,tangentAngle));
        path.poses.push_back(docking_pose_stamped_front);
    }
    //top left corner, move down
    for (int i = 0; i < point_count ; i++) {
        double xPos = wOffset;
        double yPos = height -(height * i / point_count) + hOffset;
        double tangentAngle = 270 * 2 * M_PI / 360;
        while(tangentAngle >= M_PI) {
            tangentAngle -= 2 * M_PI;
        }
        geometry_msgs::PoseStamped docking_pose_stamped_front;
        docking_pose_stamped_front.pose = pose.pose.pose;
        docking_pose_stamped_front.header = pose.header;
        docking_pose_stamped_front.pose.position.x += xPos;
        docking_pose_stamped_front.pose.position.y += yPos;
        docking_pose_stamped_front.pose.orientation = tf2::toMsg(tf2::Quaternion(0,0,tangentAngle));
        path.poses.push_back(docking_pose_stamped_front);
    }
}

void DebugBehavior::eight(nav_msgs::Path &path, double hRad, double vRad) {

}

void DebugBehavior::zigzag(nav_msgs::Path &path, double length, double height) {

}


Behavior *DebugBehavior::execute() {
    mbf_msgs::ExePathGoal exePathGoal;
    nav_msgs::Path path;

    //circle(path, 0.6);
    //ellipse(path, 0.6, 0.4);
    //circle(path, 0.5);
    //ellipse(path, 0.7, 0.3);
    //circle(path, 0.4);
    //ellipse(path, 0.75, 0.25);
    //circle(path, 0.3);
    //ellipse(path, 0.8, 0.2);
    //ellipse(path, 0.9, 0.15);
    ellipse(path, 1.0, 0.5);
    ellipse(path, 1.5, 0.3);
    //rectangle(path, 3,2);

    exePathGoal.path = path;
    exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
    exePathGoal.dist_tolerance = 0.1;
    exePathGoal.tolerance_from_action = true;
    exePathGoal.controller = "FTCPlanner";

    int cycle = 0;
    setMowerEnabled(false);
    while(!aborted && cycle<20) {
        auto result = mbfClientExePath->sendGoalAndWait(exePathGoal);
        bool success = result.state_ == actionlib::SimpleClientGoalState::SUCCEEDED;
        if (!success) {
            ROS_ERROR_STREAM("[DebugBehavior] Error executing debug path. MBF/FTCPlanner state is " << result.toString());
        }
        cycle++;
    }
    setMowerEnabled(false);

    // stop the bot for now
    stopMoving("debug safety stop");

    ROS_INFO_STREAM("[DebugBehavior] Debug finished.");
    //bool hasGps = waitForGPS();

    //if (!hasGps) {
    //    ROS_ERROR_STREAM("[DebugBehavior] Could not get GPS.");
    //}
    return &IdleBehavior::INSTANCE;
}

void DebugBehavior::enter() {
    reset();
    mower_enabled_flag_before_pause = mower_enabled_flag = paused = aborted = false;

    // Get the docking pose in map
    mower_map::GetDockingPointSrv get_docking_point_srv;
    dockingPointClient.call(get_docking_point_srv);
    docking_pose_stamped.pose = get_docking_point_srv.response.docking_pose;
    docking_pose_stamped.header.frame_id = "map";
    docking_pose_stamped.header.stamp = ros::Time::now();

    // set the robot's position to the dock
    ROS_INFO_STREAM("[DebugBehavior] Always set pose to the docks pose.");
    setRobotPose(docking_pose_stamped.pose, "debug init");
    update_actions(true);
}

void DebugBehavior::exit() {
    update_actions(false);
}

void DebugBehavior::reset() {
    gpsRequired = false;
}

bool DebugBehavior::needs_gps() {
    return gpsRequired;
}

bool DebugBehavior::waitForGPS() {
    gpsRequired = false;
    setGPS(true,"undocking finished");
    ros::Rate odom_rate(1.0);
    while (ros::ok() && !aborted) {
        if (isGpsGood()) {
            ROS_INFO("[DebugBehavior] Got good gps, let's go");
            break;
        } else {
            ROS_INFO_STREAM("[DebugBehavior] waiting for gps. current accuracy: " << getPose().position_accuracy);
            odom_rate.sleep();
        }
    }
    if (!ros::ok() || aborted) {
        return false;
    }

    // wait additional time for odometry filters to converge
    ros::Rate r(ros::Duration(config.gps_wait_time, 0));
    r.sleep();

    gpsRequired = true;

    return true;
}

DebugBehavior::DebugBehavior() {
    xbot_msgs::ActionInfo abort_debug_action;
    abort_debug_action.action_id = "abort";
    abort_debug_action.enabled = false;
    abort_debug_action.action_name = "Stop Debug";

    actions.clear();
    actions.push_back(abort_debug_action);
}

void DebugBehavior::command_home() {

}

void DebugBehavior::command_start() {

}

void DebugBehavior::command_s1() {

}

void DebugBehavior::command_s2() {

}

bool DebugBehavior::redirect_joystick() {
    return false;
}


uint8_t DebugBehavior::get_sub_state() {
    return 2;

}
uint8_t DebugBehavior::get_state() {
    return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

void DebugBehavior::update_actions(bool enable) {
    for(auto& a : actions) {
        a.enabled = enable;
    }

    // pause / resume switch. other actions are always available
    if(enable) {
        actions[0].enabled = !aborted;
    }

    registerActions("mower_logic:behavior", actions);
}

void DebugBehavior::handle_action(std::string action) {
    if (action == "mower_logic:behavior/abort") {
        ROS_INFO_STREAM("[DebugBehavior] got abort command");
        this->abort();
    } 
    update_actions(true);
}


