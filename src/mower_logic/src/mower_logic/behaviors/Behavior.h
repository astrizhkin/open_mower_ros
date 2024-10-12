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
#ifndef SRC_BEHAVIOR_H
#define SRC_BEHAVIOR_H

#include "ros/ros.h"
#include "mower_logic/MowerLogicConfig.h"
#include "mower_msgs/HighLevelStatus.h"
#include "xbot_msgs/ActionInfo.h"
#include <atomic>
#include <memory>

enum eAutoMode {
    MANUAL = 0,
    SEMIAUTO = 1,
    AUTO = 2
};

enum ePauseReason {
    PAUSE_FORCE = 0,
    PAUSE_MANUAL = 1,
    PAUSE_AUTO = 2
};

struct sSharedState {
    bool active_semiautomatic_task;
};

/**
 * Behavior definition
 */
class Behavior {

private:
    ros::Time startTime;

protected:
    std::atomic<bool> aborted;
    std::atomic<bool> paused;
    std::atomic<ePauseReason> requested_pause_reason;

    std::atomic<bool> mower_enabled_flag;
    std::atomic<bool> mower_enabled_flag_before_pause;

    std::atomic<bool> requested_continue_flag;
    std::atomic<bool> requested_pause_flag;

    std::atomic<bool> isGPSGood;
    std::atomic<uint8_t> sub_state;
 
    std::vector<xbot_msgs::ActionInfo> actions;

    double time_in_state() {
        return (ros::Time::now() - startTime).toSec();
    }

    mower_logic::MowerLogicConfig config;
    std::shared_ptr<sSharedState> shared_state;

    /**
     * Called ONCE on state enter.
     */
    virtual void enter() = 0;

public:

    virtual std::string state_name() = 0;
    virtual std::string sub_state_name() {
        return "";
    }

    bool hasGoodGPS() {
        return isGPSGood;
    }

    void setGoodGPS(bool isGood) {
        isGPSGood = isGood;
    }

    void requestContinue(ePauseReason reason) {
        if(!paused || !requested_pause_flag) {
            return;
        }
        if(requested_pause_reason != reason && reason!=ePauseReason::PAUSE_FORCE) {
            ROS_WARN_STREAM("[Behavior] Can not reset pause with reason ["<< reason <<"] different from pause set reason [" << requested_pause_reason <<"]");
            return;
        }
        requested_continue_flag = true;
    }

    void requestPause(ePauseReason reason) {
        ROS_WARN_STREAM("[Behavior] Request pause with reason ["<< reason <<"]");
        requested_pause_flag = true;
        requested_pause_reason = reason;
    }

    void setPause() {
        paused = true;
        mower_enabled_flag_before_pause = mower_enabled_flag.load();
        mower_enabled_flag = false;
    }

    void setContinue() {
        paused = false;
        requested_continue_flag = false;
        requested_pause_flag = false;
        mower_enabled_flag = mower_enabled_flag_before_pause.load();
    }

    // return true, if the mower motor should currently be running.
    bool mower_enabled() {
        return mower_enabled_flag;
    }

    void setMowerEnabled(bool enabled) {
        mower_enabled_flag = enabled;
    }

    void start(mower_logic::MowerLogicConfig &c, std::shared_ptr<sSharedState> s) {
        ROS_INFO_STREAM("");
        ROS_INFO_STREAM("");
        ROS_INFO_STREAM("--------------------------------------");
        ROS_INFO_STREAM("- Entered state: " << state_name());
        ROS_INFO_STREAM("--------------------------------------");
        aborted = false;
        this->setContinue();
        this->config = c;
        this->shared_state = std::move(s);
        startTime = ros::Time::now();
        isGPSGood = false;
        sub_state = 0;
        enter();
    }

    /**
     * Execute the behavior. This call should block until the behavior is executed fully.
     * @returns the pointer to the next behavior (can return itself).
     */
    virtual Behavior *execute() = 0;

    /**
     * Called ONCE before state exits
     */
    virtual void exit() = 0;

    /**
     * Reset the internal state of the behavior.
     */
    virtual void reset() = 0;

    /**
     * If called, save state internally and return the execute() method asap.
     * Execution should resume on the next execute() call.
     */
    void abort() {
        if(!aborted) {
            ROS_INFO_STREAM( "- Behaviour.h: abort() called");
        }
        aborted = true;
    }

    // Return true, if this state needs absolute positioning.
    // The state will be aborted if GPS is lost and resumed at some later point in time.
    virtual bool needs_gps() = 0;

    // return true to redirect joystick speeds to the controller
    virtual bool redirect_joystick() = 0;


    virtual void command_home() = 0;
    virtual void command_start() = 0;
    virtual void command_s1() = 0;
    virtual void command_s2() = 0;

    virtual uint8_t get_sub_state() = 0;
    virtual uint8_t get_state() = 0;

    virtual void handle_action(std::string action) = 0;
};

#endif //SRC_BEHAVIOR_H
