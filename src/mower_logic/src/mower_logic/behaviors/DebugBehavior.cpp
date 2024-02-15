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

DebugBehavior DebugBehavior::INSTANCE;

std::string DebugBehavior::state_name() {
    return "DEBUG";
}

Behavior *DebugBehavior::execute() {

    // get robot's current pose from odometry.
    xbot_msgs::AbsolutePose pose = getPose();
    tf2::Quaternion quat;
    tf2::fromMsg(pose.pose.pose.orientation, quat);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);


    mbf_msgs::ExePathGoal exePathGoal;

    nav_msgs::Path path;

    int point_count = 10;
    int circles_count = 5;
    double circle_radius = 0.6;
    //circle
    //ellipse
    //eight

    for (int i = 0; i <= (point_count * circles_count); i++) {
        double angle = 2 * M_PI * i / point_count;
        while(angle > M_PI) {
            angle -= M_PI;
        }
        geometry_msgs::PoseStamped docking_pose_stamped_front;
        docking_pose_stamped_front.pose = pose.pose.pose;
        docking_pose_stamped_front.header = pose.header;
        docking_pose_stamped_front.pose.position.x += cos(angle) * circle_radius;
        docking_pose_stamped_front.pose.position.y += sin(angle) * circle_radius;
        docking_pose_stamped_front.pose.orientation = tf2::toMsg(tf2::Quaternion(0,0,angle+M_PI_2));
        path.poses.push_back(docking_pose_stamped_front);
    }

    exePathGoal.path = path;
    exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
    exePathGoal.dist_tolerance = 0.1;
    exePathGoal.tolerance_from_action = true;
    exePathGoal.controller = "FTCPlanner";

    auto result = mbfClientExePath->sendGoalAndWait(exePathGoal);

    bool success = result.state_ == actionlib::SimpleClientGoalState::SUCCEEDED;

    // stop the bot for now
    stopMoving("debug safety stop");

    if (!success) {
        ROS_ERROR_STREAM("[DebugBehavior] Error during undock. MBF/FTCPlanner state is " << result.toString());
    }

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
}

void DebugBehavior::exit() {

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

void DebugBehavior::handle_action(std::string action) {
}
