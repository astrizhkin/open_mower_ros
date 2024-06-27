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

// #define VERBOSE_DEBUG   1

#include "ros/ros.h"
#include "slic3r_coverage_planner/PlanPath.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/GetDockingPointSrv.h"
#include "mower_map/SetDockingPointSrv.h"
#include "mower_map/ClearNavPointSrv.h"
#include "mower_map/SetNavPointSrv.h"
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Transform.h>
#include "mbf_msgs/ExePathAction.h"
#include "mbf_msgs/MoveBaseAction.h"
#include <nav_msgs/Odometry.h>
#include "nav_msgs/Path.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "mower_msgs/Status.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "mower_msgs/MowerControlSrv.h"
#include "mower_msgs/EmergencyModeSrv.h"
#include "ftc_local_planner/PlannerGetProgress.h"
#include <dynamic_reconfigure/server.h>
#include "mower_logic/MowerLogicConfig.h"
#include "behaviors/Behavior.h"
#include "behaviors/IdleBehavior.h"
#include "behaviors/AreaRecordingBehavior.h"
#include "mower_msgs/HighLevelControlSrv.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "mower_msgs/HighLevelStatus.h"
#include "mower_map/ClearMapSrv.h"
#include "xbot_msgs/AbsolutePose.h"
#include "xbot_positioning/GPSControlSrv.h"
#include "xbot_positioning/SetPoseSrv.h"
#include "xbot_msgs/RegisterActionsSrv.h"
#include <mutex>
#include <atomic>

ros::ServiceClient pathClient, mapClient, dockingPointClient, gpsClient, mowClient, emergencyClient, pathProgressClient, setNavPointClient, clearNavPointClient, clearMapClient, positioningClient, actionRegistrationClient;

ros::NodeHandle *n;
ros::NodeHandle *paramNh;

dynamic_reconfigure::Server<mower_logic::MowerLogicConfig> *reconfigServer;
actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *mbfClient;
actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfClientExePath;

ros::Publisher cmd_vel_pub, high_level_state_publisher;
mower_logic::MowerLogicConfig last_config;


// store some values for safety checks
ros::Time pose_time(0.0);
//2D pose
xbot_msgs::AbsolutePose last_pose2D;

ros::Time odom_time(0.0);
//3D pose
nav_msgs::Odometry last_odom3D;

ros::Time status_time(0.0);
mower_msgs::Status last_status;

ros::Time last_good_gps(0.0);

std::recursive_mutex mower_logic_mutex;

mower_msgs::HighLevelStatus high_level_status;

Behavior *currentBehavior = &IdleBehavior::INSTANCE;

/**
 * Some thread safe methods to get a copy of the logic state
 */
ros::Time getPoseTime() {
    std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
    return pose_time;
}
ros::Time getOdomTime() {
    std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
    return odom_time;
}
ros::Time getStatusTime() {
    std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
    return status_time;
}
ros::Time getLastGoodGPS() {
    std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
    return last_good_gps;
}
void setLastGoodGPS(ros::Time time) {
    std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
    last_good_gps = time;
}
mower_msgs::Status getStatus() {
    std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
    return last_status;
}

mower_logic::MowerLogicConfig getConfig() {
    std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
    return last_config;
}
void setConfig(mower_logic::MowerLogicConfig c) {
    std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
    last_config = c;
    reconfigServer->updateConfig(c);
}


xbot_msgs::AbsolutePose getPose() {
    std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
    return last_pose2D;
}

void setEmergencyMode(bool set_reset, uint8_t emergency_bit, std::string reason, ros::Duration duration);

void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo> &actions) {
    xbot_msgs::RegisterActionsSrv srv;
    srv.request.node_prefix = prefix;
    srv.request.actions = actions;

    ros::Rate retry_delay(1);
    bool success = false;
    for(int i = 0; i < 10; i++) {
        if(actionRegistrationClient.call(srv)) {
            //ROS_INFO_STREAM("[mower_logic] successfully registered actions for " << prefix);
            success = true;
            break;
        }
        ROS_ERROR_STREAM("[mower_logic] Error registering actions for " << prefix << ". Retrying.");
        retry_delay.sleep();
    }
    if (!success) {
        ROS_ERROR_STREAM("[mower_logic] Error registering actions for " << prefix << ". THIS SHOULD NEVER HAPPEN");
    }
}

void setRobotPose(geometry_msgs::Pose &pose, std::string reason) {
    // set the robot pose internally as well. othwerise we need to wait for xbot_positioning to send a new one once it has updated the internal pose.
    {
        std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
        last_pose2D.pose.pose = pose;
    }

    xbot_positioning::SetPoseSrv pose_srv;
    pose_srv.request.robot_pose = pose;
    pose_srv.request.reason = reason;

    ros::Rate retry_delay(1);
    bool success = false;
    for (int i = 0; i < 10; i++) {
        if (positioningClient.call(pose_srv)) {
//            ROS_INFO_STREAM("successfully set pose to " << pose);
            success = true;
            break;
        }
        ROS_ERROR_STREAM("[mower_logic] Error setting robot pose to " << pose << ". Retrying.");
        retry_delay.sleep();
    }

    if (!success) {
        ROS_ERROR_STREAM("[mower_logic] Error setting robot pose. Going to emergency. THIS SHOULD NEVER HAPPEN");
        setEmergencyMode(true,mower_msgs::EmergencyModeSrvRequest::EMERGENCY_POSE,"[mower_logic] Error setting robot pose",ros::Duration::ZERO);
    }
}

void poseReceived(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
    std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};

    last_pose2D = *msg;

#ifdef VERBOSE_DEBUG
    ROS_INFO("[mower_logic] pose received with accuracy %f", last_pose.position_accuracy);
#endif
    pose_time = ros::Time::now();
}

void odomReceived(const nav_msgs::Odometry::ConstPtr &msg) {
    std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};

    last_odom3D = *msg;

    odom_time = ros::Time::now();
}

void statusReceived(const mower_msgs::Status::ConstPtr &msg) {
    std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};

#ifdef VERBOSE_DEBUG
    ROS_INFO("[mower_logic] statusReceived");
#endif
    last_status = *msg;
    status_time = ros::Time::now();
}

// Abort the currently running behaviour
void abortExecution() {
    if (currentBehavior != nullptr) {
        currentBehavior->abort();
    }
}

bool setGPS(bool enabled, std::string reason) {
    xbot_positioning::GPSControlSrv gps_srv;
    gps_srv.request.gps_enabled = enabled;
    gps_srv.request.reason = reason;

    ros::Rate retry_delay(1);
    bool success = false;
    for(int i = 0; i < 10; i++) {
        if(gpsClient.call(gps_srv)) {
            //ROS_INFO_STREAM("[mower_logic] successfully set GPS to " << enabled);
            success = true;
            break;
        }
        ROS_ERROR_STREAM("[mower_logic] Error setting GPS to " << enabled << ". Retrying.");
        retry_delay.sleep();
    }

    if(!success) {
        ROS_ERROR_STREAM("[mower_logic] Error setting GPS. Going to emergency. THIS SHOULD NEVER HAPPEN");
        setEmergencyMode(true,mower_msgs::EmergencyModeSrvRequest::EMERGENCY_GPS,"[mower_logic] Error setting GPS",ros::Duration::ZERO);
    }

    return success;
}

/// @brief If the BLADE Motor is not in the requested status (enabled),we call the 
///        the mower_service/mow_enabled service to enable/disable. TODO: get feedback about spinup and delay if needed
/// @param enabled 
/// @return 
bool setMowerEnabledEx(bool enabled, float power, bool direction) {
    const auto last_config = getConfig();

    if ( !last_config.enable_mower && enabled ) {
        ROS_WARN_STREAM("[mower_logic] setMowerEnabled() - Mower should be enabled but is hard-disabled in the config.");
        enabled = false;
    }
    
    mower_msgs::MowerControlSrv mow_srv;
    mow_srv.request.mow_enabled = enabled;
    mow_srv.request.mow_power = power;
    mow_srv.request.mow_direction = direction;

    ros::Rate retry_delay(1);
    bool success = false;
    for ( int i = 0; i < 10; i++ ) {
        if( mowClient.call(mow_srv) ) {
            //ROS_INFO_STREAM("[mower_logic] successfully set mower enabled to " << enabled << " (direction " << static_cast<unsigned>(mow_srv.request.mow_direction) << ")");
            success = true;
            break;
        }
        ROS_ERROR_STREAM("[mower_logic] Error setting mower enabled to " << enabled << ". Retrying.");
        retry_delay.sleep();
    }

    if ( !success ) {
        ROS_ERROR_STREAM("[mower_logic] Error setting mower enabled. THIS SHOULD NEVER HAPPEN");
    }

    //ROS_WARN_STREAM("[mower_logic] setMowerEnabled(" << enabled << ", " << static_cast<unsigned>(mow_srv.request.mow_direction) << ") call completed within " << (ros::Time::now() - started).toSec() << "s");

// TODO: Spinup feedback & delay
/*    if (enabled) {
        ROS_INFO_STREAM("enabled mower, waiting for it to speed up");

        // TODO timeout and error
        ros::Time started = ros::Time::now();
        while (true) {
            if (status_time > started) {
                // we have a current status message, wait for mower to speed up
                bool mower_running = (last_status.speed_mow_status & 0b10);
                if (mower_running) {
                    ROS_INFO_STREAM("mower motor started");
                    return true;
                }
            }
            if (ros::Time::now() - started > ros::Duration(25.0)) {
                // mower was not able to start
                ROS_ERROR_STREAM("error starting mower motor...");
                setMowerEnabled(false);
                return false;
            }
        }
    }*/

    return true;
}

bool setMowerEnabled(bool enabled) {
    ros::Time started = ros::Time::now();
    bool direction = started.sec & 0x1; // Randomize mower direction on second
    return setMowerEnabledEx(enabled, 0.25, direction);
}

/// @brief Halt all bot movement
// it only temporary overrides autonomus velecity commands for a time configured in twist_mux config
void stopMoving(std::string reason) {
    ROS_WARN_STREAM_THROTTLE(10,"[mower_logic] stopMoving() - stopping bot movement with reason [" << reason << "]");
    geometry_msgs::Twist stop;
    stop.angular.z = 0;
    stop.linear.x = 0;
    cmd_vel_pub.publish(stop);
}

/// @brief Stop BLADE motor and any movement
/// @param emergency 

void setEmergencyMode(bool set_reset, uint8_t emergency_bit, std::string reason, ros::Duration duration) {
    if (set_reset) {
        //we immediatelly stop blade and motion
        //it must be retored to normal state by the active behavior
        setMowerEnabled(false);
        stopMoving("emergency: "+reason);
    }
    mower_msgs::EmergencyModeSrv emergencyMode;
    emergencyMode.request.set_reset = set_reset;
    emergencyMode.request.emergency_bit = emergency_bit;
    emergencyMode.request.reason = reason;
    emergencyMode.request.duration = duration;

    ros::Rate retry_delay(1);
    bool success = false;
    for ( int i = 0; i < 10; i++ ) {
        if ( emergencyClient.call(emergencyMode) ) {
            //ROS_INFO_STREAM("[mower_logic] successfully set emergency enabled to " << emergency);
            success = true;
            break;
        }
        ROS_ERROR_STREAM("[mower_logic] Error setting emergency enabled to " << set_reset << " bit " << emergency_bit << ". Retrying.");
        retry_delay.sleep();
    }

    if ( !success ) {
        ROS_ERROR_STREAM("[mower_logic] Error setting emergency. THIS SHOULD NEVER HAPPEN");
    }

}

void updateUI(const ros::TimerEvent &timer_event) {
    if ( currentBehavior ) {
        high_level_status.state_name = currentBehavior->state_name();
        high_level_status.state = (currentBehavior->get_state() & 0b11111) | (currentBehavior->get_sub_state() << mower_msgs::HighLevelStatus::SUBSTATE_SHIFT);
        high_level_status.sub_state_name = currentBehavior->sub_state_name();
    } else {
        high_level_status.state_name = "NULL";
        high_level_status.sub_state_name = "";
        high_level_status.state = mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_NULL;
    }
    high_level_state_publisher.publish(high_level_status);
}

bool isGpsGood() {
    std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
    // GPS is good if orientation is valid, we have low accuracy and we have a recent GPS update.
    // TODO: think about the "recent gps flag" since it only looks at the time. E.g. if we were standing still this would still pause even if no GPS updates are needed during standstill.
    return last_pose2D.orientation_valid && last_pose2D.position_accuracy < last_config.max_position_accuracy && (last_pose2D.flags & xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE);
}

double getNormalGravityAngle() {
    tf2::Vector3 normalGravityVector(0.0, 0.0, 9.81);
    tf2::Quaternion q;
    tf2::fromMsg(last_odom3D.pose.pose.orientation, q);
    tf2::Vector3 actualGravityVector = tf2::quatRotate(q,normalGravityVector);
    return normalGravityVector.angle(actualGravityVector);
}


/// @brief Called every 0.5s, used to control BLADE motor via mower_enabled variable and stop any movement in case of /odom and /mower/status outages
/// @param timer_event 
void checkSafety(const ros::TimerEvent &timer_event) {
    const auto last_status = getStatus();
    const auto last_config = getConfig();
    const auto last_pose = getPose();
    const auto pose_time = getPoseTime();
    const auto odom_time = getOdomTime();
    const auto status_time = getStatusTime();
    const auto last_good_gps = getLastGoodGPS();

    // only disable mower if not allowed
    if(currentBehavior != nullptr && !currentBehavior->mower_enabled()){
        setMowerEnabled(false);
    }
    

    high_level_status.emergency = last_status.emergency;
    high_level_status.is_charging = last_status.charging;

    // send to idle if emergency and we're not recording
    if( last_status.emergency ) {
        if( currentBehavior != &AreaRecordingBehavior::INSTANCE && currentBehavior != &IdleBehavior::INSTANCE ) {
            abortExecution();
        } else if( last_status.v_charge > 5.0 ) {
            // emergency and docked and idle or area recording, so it's safe to reset the emergency mode, reset it. 
            // It's safe since we won't start moving in this mode.
            setEmergencyMode(false,mower_msgs::EmergencyModeSrvRequest::EMERGENCY_LOW_BATTERY,"[mower_logic] Docked and charger battery reset",ros::Duration::ZERO);
        }
    }

    // TODO: Have a single point where we check for this timeout instead of twice (here and in the behavior)
    // check if odometry is current. If not, the GPS was bad so we stop moving.
    // Note that the mowing behavior will pause as well by itself.
    if ( ros::Time::now() - pose_time > ros::Duration(1.0) || 
         ros::Time::now() - odom_time > ros::Duration(1.0)) {
        //uncommet this if emerency is not triggered
        //setMowerEnabled(false);
        //stopMoving("pose values stopped");
        ROS_WARN_STREAM_THROTTLE(5, "[mower_logic] EMERGENCY pose/odom values stopped. pose dt: " << (ros::Time::now() - pose_time) << " odom dt: " << (ros::Time::now() - odom_time));
        setEmergencyMode(true,mower_msgs::EmergencyModeSrvRequest::EMERGENCY_POSE,"[mower_logic] pose/odom values timout",ros::Duration::ZERO);
        return;
    } else {
        //setEmergencyMode(false,mower_msgs::EmergencyModeSrvRequest::EMERGENCY_POSE,"[mower_logic] pose values ok",ros::Duration::ZERO);
    }

    double normalGravityAngle = getNormalGravityAngle();
    //convert to degree
    normalGravityAngle *= 180 / M_PI;
    if (normalGravityAngle > 30) {
        ROS_WARN_STREAM_THROTTLE(5, "[mower_logic] EMERGENCY pose gravity angle is out of bound " << (ros::Time::now() - pose_time));
        setEmergencyMode(true,mower_msgs::EmergencyModeSrvRequest::EMERGENCY_POSE,"[mower_logic] pose gravity angle is out of bound",ros::Duration::ZERO);
        return;
    } else {
        if ( normalGravityAngle < 25 ) {
            setEmergencyMode(false,mower_msgs::EmergencyModeSrvRequest::EMERGENCY_POSE,"[mower_logic] pose gravity angle is ok",ros::Duration::ZERO);
        }
    }

    // check if status is current. if not, we have a problem since it contains wheel ticks and so on.
    // Since these should never drop out, we enter emergency instead of "only" stopping
    if ( ros::Time::now() - status_time > ros::Duration(3) || last_status.ll_timeout) {
        ROS_WARN_STREAM_THROTTLE(5, "[mower_logic] EMERGENCY /mower/status values stopped. dt was: " << (ros::Time::now() - status_time));
        setEmergencyMode(true,mower_msgs::EmergencyModeSrvRequest::EMERGENCY_STATUS_TIMEOUT,"[mower_logic] /mower/status or LL timout",ros::Duration::ZERO);
        return;
    } else {
        //setEmergencyMode(false,mower_msgs::EmergencyModeSrvRequest::EMERGENCY_STATUS_TIMEOUT,"[mower_logic] /mower/status values ok",ros::Duration::ZERO);
    }

    // If the motor controllers error, we enter emergency mode in the hope to save them. They should not error.
    if (    last_status.rear_right_esc_status.status <= mower_msgs::ESCStatus::ESC_STATUS_ERROR || 
            last_status.rear_left_esc_status.status <= mower_msgs::ESCStatus ::ESC_STATUS_ERROR || 
            last_status.front_right_esc_status.status <= mower_msgs::ESCStatus ::ESC_STATUS_ERROR || 
            last_status.front_left_esc_status.status <= mower_msgs::ESCStatus ::ESC_STATUS_ERROR || 
            (last_config.enable_mower && last_status.mow_esc_status.status <= mower_msgs::ESCStatus ::ESC_STATUS_ERROR) ) {
        ROS_ERROR("[mower_logic] EMERGENCY: at least one motor control errored. errors RL: %d, RR: %d, FL: %d, FR: %d, MOW: %d",last_status.rear_left_esc_status.status,last_status.rear_right_esc_status.status,last_status.front_left_esc_status.status,last_status.front_right_esc_status.status,last_status.mow_esc_status.status);
        //set esc emergency
        setEmergencyMode(true,mower_msgs::EmergencyModeSrvRequest::EMERGENCY_ESC,"[mower_logic] motor control errored",ros::Duration::ZERO);
        return;
    } else {
        //clear esc emergency?
        //setEmergencyMode(false,mower_msgs::EmergencyModeSrvRequest::EMERGENCY_ESC,"[mower_logic] motor control ok",ros::Duration::ZERO);

        if (last_status.rear_right_esc_status.status == mower_msgs::ESCStatus::ESC_STATUS_OVERHEATED || 
                last_status.rear_left_esc_status.status == mower_msgs::ESCStatus ::ESC_STATUS_OVERHEATED || 
                last_status.front_right_esc_status.status == mower_msgs::ESCStatus ::ESC_STATUS_OVERHEATED || 
                last_status.front_left_esc_status.status == mower_msgs::ESCStatus ::ESC_STATUS_OVERHEATED ||
                last_status.mow_esc_status.status == mower_msgs::ESCStatus ::ESC_STATUS_OVERHEATED) {
            ROS_ERROR_STREAM(
                    "[mower_logic] EMERGENCY for 5 minutes: at least one motor or ESC is overheated");
            setEmergencyMode(true,mower_msgs::EmergencyModeSrvRequest::EMERGENCY_TEMPERATURE,"[mower_logic] motor or ESC is overheated", ros::Duration(300.0));
            return;
        }
    }

    // We need orientation and a positional accuracy less than configured
    bool gpsGoodNow = isGpsGood();
    if ( gpsGoodNow || last_config.ignore_gps_errors ) {
        setLastGoodGPS(ros::Time::now());
        high_level_status.gps_quality_percent = 1.0 - fmin(1.0, last_pose.position_accuracy / last_config.max_position_accuracy);
        ROS_INFO_STREAM_THROTTLE(10, "[mower_logic] GPS quality: " << high_level_status.gps_quality_percent);
    } else {
        // GPS = bad, set quality to 0
        high_level_status.gps_quality_percent = 0;
        if ( last_pose.orientation_valid ) {
            // set this if we don't even have an orientation
            high_level_status.gps_quality_percent = -1;
        }
        ROS_WARN_STREAM_THROTTLE(1,"[mower_logic] Low quality GPS");
    }

    bool gpsTimeout = ros::Time::now() - last_good_gps > ros::Duration(last_config.gps_timeout);

    if ( gpsTimeout ) {
        // GPS = bad, set quality to 0
        high_level_status.gps_quality_percent = 0;
        ROS_WARN_STREAM_THROTTLE(1,"[mower_logic] GPS timeout");
    }

    if (currentBehavior != nullptr && currentBehavior->needs_gps()) {
        // Stop the mower
        if ( gpsTimeout ) {
            setMowerEnabled(false);
            stopMoving("gps timeout");
        }
        currentBehavior->setGoodGPS(!gpsTimeout);
    }

    double battery_percent = last_status.battery_soc;
    if(battery_percent > 1.0) {
        battery_percent = 1.0;
    } else if(battery_percent < 0.0) {
        battery_percent = 0.0;
    }
    high_level_status.battery_percent = battery_percent;

    // we are in non emergency, check if we should pause. This could be empty battery, rain or hot mower motor etc.
    bool dockingNeeded = false;
    if (    last_status.v_battery < last_config.battery_empty_voltage || 
// no docking, we instead must make a pause
//            last_status.mow_esc_status.temperature_motor >= last_config.motor_hot_temperature ||
            last_config.manual_pause_mowing ) {
        dockingNeeded = true;
    }

    if (    dockingNeeded &&
            currentBehavior != &DockingBehavior::INSTANCE &&
            currentBehavior != &UndockingBehavior::RETRY_INSTANCE ) {
        ROS_WARN_STREAM_THROTTLE(1,"[mower_logic] About current behavior due to battery level or mower temperature or manual input");
        abortExecution();
    }
}

void reconfigureCB(mower_logic::MowerLogicConfig &c, uint32_t level) {
    ROS_INFO_STREAM("[mower_logic] Setting mower_logic config");
    last_config = c;
}

bool highLevelCommand(mower_msgs::HighLevelControlSrvRequest &req, mower_msgs::HighLevelControlSrvResponse &res) {
    switch(req.command) {
        case mower_msgs::HighLevelControlSrvRequest::COMMAND_HOME:
	        ROS_INFO_STREAM("[mower_logic] COMMAND_HOME");
            if(currentBehavior) {
                currentBehavior->command_home();
            }
            break;
        case mower_msgs::HighLevelControlSrvRequest::COMMAND_START:
		    ROS_INFO_STREAM("[mower_logic] COMMAND_START");
            if(currentBehavior) {
                currentBehavior->command_start();
            }
            break;
        case mower_msgs::HighLevelControlSrvRequest::COMMAND_S1:
		    ROS_INFO_STREAM("[mower_logic] COMMAND_S1");
            if(currentBehavior) {
                currentBehavior->command_s1();
            }
            break;
        case mower_msgs::HighLevelControlSrvRequest::COMMAND_S2:
	        ROS_INFO_STREAM("[mower_logic] COMMAND_S2"); 
            if(currentBehavior) {
                currentBehavior->command_s2();
            }
            break;
        case mower_msgs::HighLevelControlSrvRequest::COMMAND_DELETE_MAPS: {
            ROS_WARN_STREAM("[mower_logic] COMMAND_DELETE_MAPS");
            if (currentBehavior != &AreaRecordingBehavior::INSTANCE && currentBehavior != &IdleBehavior::INSTANCE &&
                currentBehavior !=
                nullptr) {
                ROS_ERROR_STREAM("[mower_logic] Deleting maps is only allowed during IDLE or AreaRecording!");
                return true;
            }
            mower_map::ClearMapSrv clear_map_srv;
            // TODO check result
            clearMapClient.call(clear_map_srv);

            // Abort the current behavior. Idle will refresh and go to AreaRecorder, AreaRecorder will to to Idle wich will go to a fresh AreaRecorder
            currentBehavior->abort();
        }
            break;
        case mower_msgs::HighLevelControlSrvRequest::COMMAND_RESET_EMERGENCY:
            ROS_WARN_STREAM("[mower_logic] COMMAND_RESET_EMERGENCY");
            setEmergencyMode(false,mower_msgs::EmergencyModeSrvRequest::EMERGENCY_ALL,"[mower_logic] COMMAND_RESET_EMERGENCY",ros::Duration::ZERO);
            break;
    }
    return true;
}

void actionReceived(const std_msgs::String::ConstPtr &action) {
    if (currentBehavior) {
        currentBehavior->handle_action(action->data);
    }
}

void joyVelReceived(const geometry_msgs::Twist::ConstPtr &joy_vel) {
    if (currentBehavior && currentBehavior->redirect_joystick()) {
        ROS_INFO_STREAM("[mower_logic] redirect joystic cmd " << joy_vel->linear.x << ", " << joy_vel->angular.z);
        cmd_vel_pub.publish(joy_vel);
    }
}

void joyMowerReceived(const std_msgs::Float32::ConstPtr &joy_mower) {
    if (currentBehavior && currentBehavior->redirect_joystick()) {
        ROS_INFO_STREAM("[mower_logic] joy mower cmd " << joy_mower->data);
        float power = (joy_mower->data + 1.0)/2.0;
        setMowerEnabledEx(true, power, true);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mower_logic");

    n = new ros::NodeHandle();
    paramNh = new ros::NodeHandle("~");

    boost::recursive_mutex mutex;

    reconfigServer = new dynamic_reconfigure::Server<mower_logic::MowerLogicConfig>(mutex, *paramNh);
    reconfigServer->setCallback(reconfigureCB);

    cmd_vel_pub = n->advertise<geometry_msgs::Twist>("/logic_vel", 1);

    ros::Publisher path_pub;

    path_pub = n->advertise<nav_msgs::Path>("mower_logic/mowing_path", 100, true);
    high_level_state_publisher = n->advertise<mower_msgs::HighLevelStatus>("mower_logic/current_state", 100, true);

    pathClient = n->serviceClient<slic3r_coverage_planner::PlanPath>(
            "slic3r_coverage_planner/plan_path");
    mapClient = n->serviceClient<mower_map::GetMowingAreaSrv>(
            "mower_map_service/get_mowing_area");
    clearMapClient = n->serviceClient<mower_map::ClearMapSrv>(
            "mower_map_service/clear_map");

    gpsClient = n->serviceClient<xbot_positioning::GPSControlSrv>(
            "xbot_positioning/set_gps_state");
    positioningClient = n->serviceClient<xbot_positioning::SetPoseSrv>(
            "xbot_positioning/set_robot_pose");
    actionRegistrationClient = n->serviceClient<xbot_msgs::RegisterActionsSrv>(
            "xbot/register_actions");


    mowClient = n->serviceClient<mower_msgs::MowerControlSrv>(
            "mower_service/mow_enabled");
    emergencyClient = n->serviceClient<mower_msgs::EmergencyModeSrv>(
            "mower_service/emergency");

    dockingPointClient = n->serviceClient<mower_map::GetDockingPointSrv>(
            "mower_map_service/get_docking_point");

    pathProgressClient = n->serviceClient<ftc_local_planner::PlannerGetProgress>(
            "/move_base_flex/FTCPlanner/planner_get_progress");

    setNavPointClient = n->serviceClient<mower_map::SetNavPointSrv>(
            "mower_map_service/set_nav_point");
    clearNavPointClient = n->serviceClient<mower_map::ClearNavPointSrv>(
            "mower_map_service/clear_nav_point");


    mbfClient = new actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>("/move_base_flex/move_base");
    mbfClientExePath = new actionlib::SimpleActionClient<mbf_msgs::ExePathAction>("/move_base_flex/exe_path");

    ros::Subscriber status_sub = n->subscribe("/mower/status", 0, statusReceived, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber pose_sub = n->subscribe("/xbot_positioning/xb_pose", 0, poseReceived, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber odom_sub = n->subscribe("/xbot_positioning/odom_out", 0, odomReceived, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber joy_move_cmd = n->subscribe("/joy_vel", 0, joyVelReceived, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber joy_mower_cmd = n->subscribe("/joy_mower", 0, joyMowerReceived, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber action = n->subscribe("xbot/action", 0, actionReceived, ros::TransportHints().tcpNoDelay(true));

    ros::ServiceServer high_level_control_srv = n->advertiseService("mower_service/high_level_control", highLevelCommand);

    ros::AsyncSpinner asyncSpinner(1);
    asyncSpinner.start();

    ros::Rate r(1.0);

    ROS_INFO("[mower_logic] Waiting for a status message");
    while (status_time == ros::Time(0.0)) {
        if (!ros::ok()) {
            delete (reconfigServer);
            delete (mbfClient);
            delete (mbfClientExePath);
            return 1;
        }
        r.sleep();
    }
    ROS_INFO("[mower_logic] Waiting for a pose message");
    while (pose_time == ros::Time(0.0)) {
        if (!ros::ok()) {
            delete (reconfigServer);
            delete (mbfClient);
            delete (mbfClientExePath);
            return 1;
        }
        r.sleep();
    }
    ROS_INFO("[mower_logic] Waiting for a odom message");
    while (odom_time == ros::Time(0.0)) {
        if (!ros::ok()) {
            delete (reconfigServer);
            delete (mbfClient);
            delete (mbfClientExePath);
            return 1;
        }
        r.sleep();
    }

    ROS_INFO("[mower_logic] Waiting for emergency service");
    if (!emergencyClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("[mower_logic] Emergency server not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);

        return 1;
    }

    ROS_INFO("[mower_logic] Waiting for path server");
    if (!pathClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("[mower_logic] Path service not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);

        return 1;
    }

    ROS_INFO("[mower_logic] Waiting for mower service");
    if (!mowClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("[mower_logic] Mower service not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);

        return 1;
    }

    ROS_INFO("[mower_logic] Waiting for gps service");
    if (!gpsClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("[mower_logic] GPS service not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);

        return 1;
    }

    ROS_INFO("[mower_logic] Waiting for positioning service");
    if (!positioningClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("[mower_logic] positioning service not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);

        return 1;
    }

    ROS_INFO("[mower_logic] Waiting for map server");
    if (!mapClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("[mower_logic] Map server service not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);
        return 2;
    }

    ROS_INFO("[mower_logic] Waiting for docking point server");
    if (!dockingPointClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("[mower_logic] Docking server service not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);
        return 2;
    }

    ROS_INFO("[mower_logic] Waiting for nav point server");
    if (!setNavPointClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("[mower_logic] Set Nav Point server service not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);
        return 2;
    }
    
    ROS_INFO("[mower_logic] Waiting for clear nav point server");
    if (!clearNavPointClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("[mower_logic] Clear Nav Point server service not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);
        return 2;
    }

    ROS_INFO("[mower_logic] Waiting for move base flex");
    if (!mbfClient->waitForServer(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("[mower_logic] Move base flex not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);
        return 3;
    }

    ROS_INFO("[mower_logic] Waiting for mowing path progress server");
    if (!pathProgressClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("[mower_logic] FTCLocalPlanner progress server not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);
        return 3;
    }

    ros::Time started = ros::Time::now();
    while((ros::Time::now() - started).toSec() < 10.0) {
        ROS_INFO_STREAM("[mower_logic] Waiting for an emergency status message");
        r.sleep();
        if(last_status.emergency) {
            ROS_INFO_STREAM("[mower_logic] Got emergency, resetting it");
            setEmergencyMode(false,mower_msgs::EmergencyModeSrvRequest::EMERGENCY_ALL,"[mower_logic] Startup expected emergency reset",ros::Duration::ZERO);
            break;
        }
    }

    ROS_INFO("[mower_logic] Got all servers, we can mow");

    ros::Timer safety_timer = n->createTimer(ros::Duration(0.5), checkSafety);
    ros::Timer ui_timer = n->createTimer(ros::Duration(1.0), updateUI);

    // release emergency if it was set
    setEmergencyMode(false,mower_msgs::EmergencyModeSrvRequest::EMERGENCY_ALL,"[mower_logic] Final startup emergency reset",ros::Duration::ZERO);

    // initialise the shared state object to be passed into the behaviors
    auto shared_state = std::make_shared<sSharedState>();
    shared_state->active_semiautomatic_task = false;

    // Behavior execution loop
    while (ros::ok()) {
        if (currentBehavior != nullptr) {
            currentBehavior->start(last_config, shared_state);
            Behavior *newBehavior = currentBehavior->execute();
            currentBehavior->exit();
            currentBehavior = newBehavior;
        } else {
            high_level_status.state_name = "NULL";
            high_level_status.state = mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_NULL;
            high_level_state_publisher.publish(high_level_status);
            // we have no defined behavior, set emergency
            ROS_ERROR_STREAM("[mower_logic] null behavior - emergency mode");
            setEmergencyMode(true,mower_msgs::EmergencyModeSrvRequest::EMERGENCY_BEHAVIOR,"[mower_logic] null behavior",ros::Duration::ZERO);
            ros::Rate r(1.0);
            r.sleep();
        }
    }

    delete (n);
    delete (paramNh);
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);
    return 0;
}

