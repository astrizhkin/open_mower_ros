//
// Created by Clemens Elflein on 15.03.22.
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
#include <dynamic_reconfigure/client.h>
#include <geometry_msgs/Twist.h>
#include <hoverboard_driver/HoverboardStateStamped.h>
#include <mower_msgs/Status.h>
#include <sensor_msgs/Joy.h>
#include <serial/serial.h>
#include <xbot_msgs/WheelTick.h>
#include <xesc_driver/xesc_driver.h>
#include <xesc_msgs/XescStateStamped.h>

#include <algorithm>
#include <bitset>

#include "COBS.h"
#include "boost/crc.hpp"
#include "ll_datatypes.h"
#include "mower_logic/MowerLogicConfig.h"
#include "mower_msgs/EmergencyModeSrv.h"
#include "mower_msgs/HighLevelControlSrv.h"
#include "mower_msgs/HighLevelStatus.h"
#include "mower_msgs/MowerControlSrv.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

ros::Publisher status_pub;
ros::Publisher wheel_tick_pub;

ros::Publisher sensor_imu_pub;
ros::Publisher sensor_mag_pub;

ros::Publisher cmd_vel_safe_pub;

COBS cobs;

// 8 bits of high level emergency set/reset in ROS
uint8_t emergency_high_level_bits = 0;
std::string emergency_high_level_reasons[] = {"", "", "", "", "", "", "", ""};
ros::Time emergency_high_level_end[] = {ros::Time::ZERO, ros::Time::ZERO, ros::Time::ZERO, ros::Time::ZERO,
                                        ros::Time::ZERO, ros::Time::ZERO, ros::Time::ZERO, ros::Time::ZERO};

// True, if the LL board thinks there should be an emergency
uint8_t emergency_low_level_bits = 0;

// True, if we can send to the low level board
bool allow_send = false;

// Current speeds (duty cycle) for the three ESCs
geometry_msgs::Twist last_cmd_twist;
ros::Time last_cmd_twist_time(0.0);
float speed_mow = 0;
uint8_t mower_enabled = 0;
uint8_t mower_direction = 0;

// Ticks / m and wheel distance for this robot
double wheel_radius_m = 0.0;
double wheel_separation_m = 0.0;
double cmd_vel_timout = 0.0;

bool mower_esc_enabled = false;

// LL/HL configuration
struct ll_high_level_config llhl_config;

dynamic_reconfigure::Client<mower_logic::MowerLogicConfig> *reconfigClient;
mower_logic::MowerLogicConfig mower_logic_config;

// Serial port and buffer for the low level connection
serial::Serial serial_port;
uint8_t out_buf[1000];

boost::crc_ccitt_type crc;

// mower_msgs::HighLevelStatus last_high_level_status;
// ros::Time last_high_level_status_time(0.0);

xesc_driver::XescDriver *mow_xesc_interface;
hoverboard_driver::HoverboardStateStamped last_rear_status;
hoverboard_driver::HoverboardStateStamped last_front_status;
xesc_msgs::XescStateStamped last_mow_status;

std::mutex ll_status_mutex;
struct ll_status last_ll_status = {0};
ros::Time last_ll_status_time(0.0);
ros::Time last_ll_status_esc_enabled(0.0);

sensor_msgs::MagneticField sensor_mag_msg;
sensor_msgs::Imu sensor_imu_msg;
double imu_gyro_multiplier[] = {0.0, 0.0, 0.0};
int imu_gyro_idx[] = {-1, -1, -1};
double imu_accel_multiplier[] = {0.0, 0.0, 0.0};
int imu_accel_idx[] = {-1, -1, -1};

ros::ServiceClient highLevelClient;

void sendLLMessage(uint8_t *msg, size_t size) {
  crc.reset();
  crc.process_bytes(msg, size - 2);
  unsigned short crcVal = crc.checksum();
  msg[size - 1] = (crcVal >> 8) & 0xFF;
  msg[size - 2] = crcVal & 0xFF;
  // ROS_INFO_STREAM("[mower_comms] sendLL CRC " << crcVal);
  size_t encoded_size = cobs.encode(msg, size, out_buf);
  out_buf[encoded_size] = 0;
  encoded_size++;

  if (serial_port.isOpen() && allow_send) {
    try {
      serial_port.write(out_buf, encoded_size);
    } catch (std::exception &e) {
      ROS_ERROR_STREAM("[mower_comms] Error writing to serial port");
    }
  }
}

bool isEmergency() {
  return emergency_high_level_bits > 0 || emergency_low_level_bits > 0;
}

bool isTemporaryEmergency() {
  for (uint8_t bit = 0; bit < 8; bit++) {
    if (emergency_high_level_bits & (1 << bit) && emergency_high_level_end[bit].isZero()) {
      return false;
    }
  }
  return true && (emergency_low_level_bits == EMERGENCY_HIGH_LEVEL || emergency_low_level_bits == 0);
}

void updateEmergencyBits() {
  for (uint8_t bit = 0; bit < 8; bit++) {
    if (emergency_high_level_bits & (1 << bit) && !emergency_high_level_end[bit].isZero() &&
        emergency_high_level_end[bit] < ros::Time::now()) {
      ROS_WARN_STREAM("[mower_comms] Autoreset emergency bit " << bit << " having reason "
                                                               << emergency_high_level_reasons[bit]);
      emergency_high_level_bits &= ~(1 << bit);
      emergency_high_level_end[bit] = ros::Time::ZERO;
    }
  }

  if ((last_ll_status.emergency_bitmask & (1 << EMERGENCY_BUTTON1_BIT)) > 0 &&
      (emergency_high_level_bits & (1 << mower_msgs::EmergencyModeSrvRequest::EMERGENCY_ESC)) > 0) {
    ROS_WARN_STREAM("[mower_comms] Autoreset ESC emergency due to Emergency Button 1 pressed");
    emergency_high_level_bits &= ~(1 << mower_msgs::EmergencyModeSrvRequest::EMERGENCY_ESC);
    emergency_high_level_end[mower_msgs::EmergencyModeSrvRequest::EMERGENCY_ESC] = ros::Time::ZERO;
  }
}

void publishActuators() {
  geometry_msgs::Twist execute_vel;
  execute_vel.linear.x = last_cmd_twist.linear.x;
  execute_vel.angular.z = last_cmd_twist.angular.z;
  // emergency or timeout -> send 0 speeds
  if (isEmergency()) {
    // TODO: publish speed topic?
    execute_vel.linear.x = 0;
    execute_vel.angular.z = 0;
    speed_mow = 0;
  }
  if (ros::Time::now() - last_cmd_twist_time > ros::Duration(cmd_vel_timout)) {
    // TODO: publish speed topic?
    execute_vel.linear.x = 0;
    execute_vel.angular.z = 0;
  }
  if (ros::Time::now() - last_cmd_twist_time > ros::Duration(25.0)) {
    // TODO: publish speed topic?
    execute_vel.linear.x = 0;
    execute_vel.angular.z = 0;
    speed_mow = 0;
  }

  if (mow_xesc_interface) {
    mow_xesc_interface->setDutyCycle(speed_mow);
  }
  cmd_vel_safe_pub.publish(execute_vel);

  struct ll_heartbeat heartbeat = {.type = PACKET_ID_LL_HEARTBEAT,
                                   // send high level emergency bits
                                   .emergency_high_level = emergency_high_level_bits};
  sendLLMessage((uint8_t *)&heartbeat, sizeof(struct ll_heartbeat));
}

void convertXescStatus(mower_msgs::Status &status_msg, xesc_msgs::XescStateStamped &vesc_status,
                       mower_msgs::ESCStatus &ros_esc_status) {
  uint8_t statusNoTemperatures = vesc_status.state.fault_code & ~(xesc_msgs::XescState::XESC_FAULT_OVERTEMP_MOTOR |
                                                                  xesc_msgs::XescState::XESC_FAULT_OVERTEMP_PCB);

  if (!status_msg.esc_power || (ros::Time::now() - last_ll_status_esc_enabled).toSec() < 5.0) {
    // report esc off status when disabled or started less than 5 seconds ago to prevent report disconnected status
    ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OFF;
  } else if (vesc_status.state.connection_state != xesc_msgs::XescState::XESC_CONNECTION_STATE_CONNECTED &&
             vesc_status.state.connection_state !=
                 xesc_msgs::XescState::XESC_CONNECTION_STATE_CONNECTED_INCOMPATIBLE_FW) {
    // ESC is disconnected
    ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
  } else if (statusNoTemperatures) {
    ROS_ERROR_STREAM_THROTTLE(1, "[mower_comms] xESC controller fault code: " << vesc_status.state.fault_code);
    // ESC has a fault
    ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_ERROR;
  } else {
    // ESC is OK
    ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;

    // If evrything looks ok at the moment, check temperatures
    if (vesc_status.state.fault_code & xesc_msgs::XescState::XESC_FAULT_OVERTEMP_PCB) {
      ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OVERHEATED;
    }
    if (vesc_status.state.fault_code & xesc_msgs::XescState::XESC_FAULT_OVERTEMP_MOTOR) {
      ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OVERHEATED;
    }
  }

  ros_esc_status.tacho = vesc_status.state.tacho;
  ros_esc_status.rpm = vesc_status.state.rpm;
  ros_esc_status.current = vesc_status.state.current_input;
  ros_esc_status.temperature_motor = vesc_status.state.temperature_motor;
  ros_esc_status.temperature_pcb = vesc_status.state.temperature_pcb;
}

void convertHoverboardStatus(mower_msgs::Status &status_msg, hoverboard_driver::HoverboardStateStamped &state_msg,
                             mower_msgs::ESCStatus &ros_esc_left_status, mower_msgs::ESCStatus &ros_esc_right_status) {
  uint8_t statusNoTemperaturesNoBattery =
      state_msg.state.status &
      ~(hoverboard_driver::HoverboardState::STATUS_PCB_TEMP_WARN |
        hoverboard_driver::HoverboardState::STATUS_PCB_TEMP_ERR |
        hoverboard_driver::HoverboardState::STATUS_LEFT_MOTOR_TEMP_ERR |
        hoverboard_driver::HoverboardState::STATUS_RIGHT_MOTOR_TEMP_ERR |
        hoverboard_driver::HoverboardState::STATUS_BATTERY_L1 | hoverboard_driver::HoverboardState::STATUS_BATTERY_L2);

  if (!status_msg.esc_power || (ros::Time::now() - last_ll_status_esc_enabled).toSec() < 3.0) {
    // report esc off status when disabled or started less than 2 seconds ago to prevent report disconnected status
    ros_esc_left_status.status = mower_msgs::ESCStatus::ESC_STATUS_OFF;
    ros_esc_right_status.status = mower_msgs::ESCStatus::ESC_STATUS_OFF;
  } else if (state_msg.state.connection_state !=
             hoverboard_driver::HoverboardState::HOVERBOARD_CONNECTION_STATE_CONNECTED
             //&& vesc_status.state.connection_state !=
             //xesc_msgs::XescState::XESC_CONNECTION_STATE_CONNECTED_INCOMPATIBLE_FW
  ) {
    // ESC is disconnected, the bad status that never should happen
    ROS_ERROR_STREAM_THROTTLE(
        1, "[mower_comms] Hoverborad connection status: " << (int)state_msg.state.connection_state
                                                          << " status code: " << state_msg.state.status);
    ros_esc_left_status.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
    ros_esc_right_status.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
  } else if (statusNoTemperaturesNoBattery) {
    ROS_ERROR_STREAM_THROTTLE(1, "[mower_comms] Hoverborad controller status code: " << state_msg.state.status);
    // ESC has a fault
    ros_esc_left_status.status = mower_msgs::ESCStatus::ESC_STATUS_ERROR;
    ros_esc_right_status.status = mower_msgs::ESCStatus::ESC_STATUS_ERROR;
    // FIXME!!! temporary disable hoverboard errors
    // ros_esc_left_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
    // ros_esc_right_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
  } else {
    // ESC is OK but we will check temperatures
    ros_esc_left_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
    ros_esc_right_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;

    // If evrything looks ok at the moment, check temperatures
    if (state_msg.state.status & hoverboard_driver::HoverboardState::STATUS_PCB_TEMP_WARN) {
      ROS_WARN_STREAM_THROTTLE(10, "[mower_comms] Motor controller PCB temerature warning");
    }
    if (state_msg.state.status & hoverboard_driver::HoverboardState::STATUS_PCB_TEMP_ERR) {
      ros_esc_left_status.status = mower_msgs::ESCStatus::ESC_STATUS_OVERHEATED;
      ros_esc_right_status.status = mower_msgs::ESCStatus::ESC_STATUS_OVERHEATED;
    }
    if (state_msg.state.status & hoverboard_driver::HoverboardState::STATUS_LEFT_MOTOR_TEMP_ERR) {
      ROS_WARN_STREAM_THROTTLE(10, "[mower_comms] Left Motor temerature error: " << state_msg.state.motorL_temp);
      // ros_esc_left_status.status = mower_msgs::ESCStatus::ESC_STATUS_OVERHEATED;
    }
    if (state_msg.state.status & hoverboard_driver::HoverboardState::STATUS_RIGHT_MOTOR_TEMP_ERR) {
      ROS_WARN_STREAM_THROTTLE(10, "[mower_comms] Right Motor temerature error: " << state_msg.state.motorR_temp);
      // ros_esc_right_status.status = mower_msgs::ESCStatus::ESC_STATUS_OVERHEATED;
    }

    // and log battery warings
    if (state_msg.state.status & hoverboard_driver::HoverboardState::STATUS_BATTERY_L1 ||
        state_msg.state.status & hoverboard_driver::HoverboardState::STATUS_BATTERY_L2) {
      ROS_WARN_STREAM("[mower_comms] Motor controller reports battery warning");
    }
  }

  // if (abs(state_msg.state.cmdL - state_msg.state.speedL_meas) > state_msg.state.cmdL * 0.2 ) {
  //     ROS_WARN_STREAM_THROTTLE(1, "[mower_comms] Motor L stall/overrun detected cmd=" << state_msg.state.cmdL << "
  //     spd=" << state_msg.state.speedL_meas);
  // }
  // if (abs(state_msg.state.cmdR - state_msg.state.speedR_meas) > state_msg.state.cmdR * 0.2 ) {
  //     ROS_WARN_STREAM_THROTTLE(1, "[mower_comms] Motor R stall/overrun detected cmd=" << state_msg.state.cmdR << "
  //     spd=" << state_msg.state.speedR_meas);
  // }

  // TODO check .tacho compatibility with howerboard .wheelX_cnt
  ros_esc_left_status.tacho = state_msg.state.wheelL_cnt;
  ros_esc_left_status.current = state_msg.state.currL_meas;
  ros_esc_left_status.temperature_motor = state_msg.state.motorL_temp;
  ros_esc_left_status.temperature_pcb = state_msg.state.boardTemp;

  ros_esc_right_status.tacho = state_msg.state.wheelR_cnt;
  ros_esc_right_status.current = state_msg.state.currR_meas;
  ros_esc_right_status.temperature_motor = state_msg.state.motorR_temp;
  ros_esc_right_status.temperature_pcb = state_msg.state.boardTemp;
}

void publishStatus() {
  mower_msgs::Status status_msg;
  status_msg.stamp = ros::Time::now();

  if (last_ll_status.status_bitmask & (1 << STATUS_INIT_BIT)) {
    // LL OK, fill the message
    status_msg.mower_status = mower_msgs::Status::MOWER_STATUS_OK;
  } else {
    // LL initializing
    status_msg.mower_status = mower_msgs::Status::MOWER_STATUS_INITIALIZING;
  }
  double llAge = (status_msg.stamp - last_ll_status_time).toSec();

  status_msg.raspberry_pi_power = (last_ll_status.status_bitmask & (1 << STATUS_RASPI_POWER_BIT)) != 0;
  status_msg.charging = (last_ll_status.status_bitmask & (1 << STATUS_CHARGING_BIT)) != 0;
  status_msg.esc_power = (last_ll_status.status_bitmask & (1 << STATUS_ESC_ENABLED_BIT)) != 0;
  status_msg.rain_detected = (last_ll_status.status_bitmask & (1 << STATUS_RAIN_BIT)) != 0;
  status_msg.uss_timeout = (last_ll_status.status_bitmask & (1 << STATUS_USS_TIMEOUT_BIT)) != 0;
  status_msg.imu_timeout = (last_ll_status.status_bitmask & (1 << STATUS_IMU_TIMEOUT_BIT)) != 0;
  status_msg.battery_empty = (last_ll_status.status_bitmask & (1 << STATUS_BATTERY_EMPTY_BIT)) != 0;
  status_msg.bms_timeout = (last_ll_status.status_bitmask & (1 << STATUS_BMS_TIMEOUT_BIT)) != 0;
  status_msg.ll_timeout = llAge > 1.0;
  status_msg.mow_enabled = mower_enabled;

  for (uint8_t i = 0; i < 5; i++) {
    status_msg.uss_ranges[i] = last_ll_status.uss_ranges_m[i];
    status_msg.uss_age_ms[i] = last_ll_status.uss_age_ms[i];
  }

  // overwrite emergency with the LL value.
  emergency_low_level_bits = last_ll_status.emergency_bitmask;
  if (emergency_low_level_bits > 0) {
    ROS_ERROR_STREAM_THROTTLE(1, "[mower_comms] Low Level Emergency. Bitmask: " << (int)last_ll_status.emergency_bitmask
                                                                                << " Age " << llAge << "s");
  }
  if (emergency_high_level_bits > 0) {
    ROS_ERROR_STREAM_THROTTLE(1, "[mower_comms] High Level Emergency. Bitmask: " << (int)emergency_high_level_bits);
  }

  // True, if high or low level emergency condition is present
  status_msg.emergency = isEmergency();
  status_msg.temporary_emergency = isTemporaryEmergency();

  status_msg.v_battery = last_ll_status.v_system;
  status_msg.v_charge = last_ll_status.v_charge;
  status_msg.battery_current = last_ll_status.battery_current;
  status_msg.battery_soc = last_ll_status.batt_percentage;

  if (mow_xesc_interface && status_msg.esc_power) {
    mow_xesc_interface->getStatus(last_mow_status);
  } else {
    last_mow_status.header.stamp = ros::Time::now();
    last_mow_status.state.connection_state = xesc_msgs::XescState::XESC_CONNECTION_STATE_DISCONNECTED;
  }

  convertXescStatus(status_msg, last_mow_status, status_msg.mow_esc_status);
  convertHoverboardStatus(status_msg, last_rear_status, status_msg.rear_left_esc_status,
                          status_msg.rear_right_esc_status);
  convertHoverboardStatus(status_msg, last_front_status, status_msg.front_left_esc_status,
                          status_msg.front_right_esc_status);

  // publish LL message
  double rear_status_age_s_double = (ros::Time::now() - last_rear_status.header.stamp).toSec();
  uint8_t rear_status_age_s_uint8_t = rear_status_age_s_double;
  if (rear_status_age_s_double > UINT8_MAX ||
      last_rear_status.state.connection_state ==
          hoverboard_driver::HoverboardState::HOVERBOARD_CONNECTION_STATE_DISCONNECTED) {
    rear_status_age_s_uint8_t = UINT8_MAX;
  }
  double front_status_age_s_double = (ros::Time::now() - last_front_status.header.stamp).toSec();
  uint8_t front_status_age_s_uint8_t = front_status_age_s_double;
  if (front_status_age_s_double > UINT8_MAX ||
      last_front_status.state.connection_state ==
          hoverboard_driver::HoverboardState::HOVERBOARD_CONNECTION_STATE_DISCONNECTED) {
    front_status_age_s_uint8_t = UINT8_MAX;
  }
  double mow_status_age_s_double = (ros::Time::now() - last_mow_status.header.stamp).toSec();
  uint8_t mow_status_age_s_uint8_t = mow_status_age_s_double;
  if (mow_status_age_s_double > UINT8_MAX ||
      last_mow_status.state.connection_state == xesc_msgs::XescState::XESC_CONNECTION_STATE_DISCONNECTED) {
    mow_status_age_s_uint8_t = UINT8_MAX;
  }
  struct ll_motor_state ll_motor_state = {
      .type = PACKET_ID_LL_MOTOR_STATE,
      .status = {last_rear_status.state.status, last_front_status.state.status, last_mow_status.state.fault_code},
      .status_age_s = {rear_status_age_s_uint8_t, front_status_age_s_uint8_t, mow_status_age_s_uint8_t}};
  sendLLMessage((uint8_t *)&ll_motor_state, sizeof(struct ll_motor_state));

  // publis topic status
  status_pub.publish(status_msg);

  xbot_msgs::WheelTick wheel_tick_msg;
  wheel_tick_msg.valid_wheels = xbot_msgs::WheelTick::WHEEL_VALID_FL | xbot_msgs::WheelTick::WHEEL_VALID_FR |
                                xbot_msgs::WheelTick::WHEEL_VALID_RL | xbot_msgs::WheelTick::WHEEL_VALID_RR;
  wheel_tick_msg.wheel_pos_to_tick_factor = 0;  // TODO: pass it for F9R
  wheel_tick_msg.wheel_radius = wheel_radius_m;
  // wheel_tick_msg.wheel_separation = wheel_separation_m;
  wheel_tick_msg.stamp = status_msg.stamp;
  // check compatibility .wheel_ticks_rl and hoverboard .wheelX_cnt (previosuly was tacho_absolute)

  wheel_tick_msg.wheel_pos_fl = last_front_status.state.wheelL_cnt;
  // wheel_tick_msg.wheel_speed_fl = last_front_status.state.speedL_meas;
  wheel_tick_msg.wheel_direction_fl = last_front_status.state.speedL_meas > 0;
  wheel_tick_msg.wheel_pos_fr = last_front_status.state.wheelR_cnt;
  // wheel_tick_msg.wheel_speed_fr = last_front_status.state.speedR_meas;
  wheel_tick_msg.wheel_direction_fr = last_front_status.state.speedR_meas > 0;

  wheel_tick_msg.wheel_pos_rl = last_rear_status.state.wheelL_cnt;
  // wheel_tick_msg.wheel_speed_rl = last_rear_status.state.speedL_meas;
  wheel_tick_msg.wheel_direction_rl = last_rear_status.state.speedL_meas > 0;
  wheel_tick_msg.wheel_pos_rr = last_rear_status.state.wheelR_cnt;
  // wheel_tick_msg.wheel_speed_rr = last_rear_status.state.speedR_meas;
  wheel_tick_msg.wheel_direction_rr = last_rear_status.state.speedR_meas > 0;

  wheel_tick_pub.publish(wheel_tick_msg);
}

std::string getHallConfigsString(const HallConfig *hall_configs, const size_t size) {
  std::string str;

  // Parse hall_configs and build a readable string
  for (size_t i = 0; i < size; i++) {
    if (str.length()) str.append(", ");
    if (hall_configs->active_low) str.append("!");
    switch (hall_configs->mode) {
      case HallMode::OFF: str.append("I"); break;
      case HallMode::LIFT_TILT: str.append("L"); break;
      case HallMode::STOP: str.append("S"); break;
      case HallMode::UNDEFINED: str.append("U"); break;
      default: break;
    }
    hall_configs++;
  }

  return str;
}

void publishLowLevelConfig(const uint8_t pkt_type) {
  if (!serial_port.isOpen() || !allow_send) return;

  // Prepare the pkt
  size_t size = sizeof(struct ll_high_level_config) + 3;  // +1 type, +2 crc
  uint8_t buf[size];

  // Send config and request a config answer
  buf[0] = pkt_type;

  // Copy our live config into the message (behind type)
  memcpy(&buf[1], &llhl_config, sizeof(struct ll_high_level_config));

  // Member access to buffer
  struct ll_high_level_config *buf_config = (struct ll_high_level_config *)&buf[1];

  // Let's be verbose for easier follow-up
  ROS_INFO(
      "Send ll_high_level_config packet %#04x\n"
      "\t options{dfp_is_5v=%d, background_sounds=%d, ignore_charging_current=%d},\n"
      "\t v_charge_cutoff=%f, i_charge_cutoff=%f,\n"
      "\t v_battery_cutoff=%f, v_battery_empty=%f, v_battery_full=%f,\n"
      "\t lift_period=%d, tilt_period=%d,\n"
      "\t shutdown_esc_max_pitch=%d,\n"
      "\t language=\"%.2s\", volume=%d\n"
      "\t hall_configs=\"%s\"",
      buf[0], (int)buf_config->options.dfp_is_5v, (int)buf_config->options.background_sounds,
      (int)buf_config->options.ignore_charging_current, buf_config->v_charge_cutoff, buf_config->i_charge_cutoff,
      buf_config->v_battery_cutoff, buf_config->v_battery_empty, buf_config->v_battery_full, buf_config->lift_period,
      buf_config->tilt_period, buf_config->shutdown_esc_max_pitch, buf_config->language, buf_config->volume,
      getHallConfigsString(buf_config->hall_configs, MAX_HALL_INPUTS).c_str());

  //Send
  sendLLMessage(buf, sizeof(struct ll_high_level_config));
}

/**
 * @brief A simple config tracker (struct-class) for managing lost response packets as well as simpler handling of
 * LowLevel reboots or flash period.
 */
struct {
  ros::Time last_config_req;    // Time when last config request was sent
  unsigned int tries_left = 0;  // Remaining request tries before giving up

  void ackResponse() {  // Call this on receive of a response packet to stop monitoring
    tries_left = 0;
  };
  void setDirty() {  // Call this for indicating that config packet need to be resend, i.e. die to LL-reboot
    tries_left = 5;
  };
  void check() {
    if (!tries_left ||                                            // No request tries left (probably old LL-FW)
        !serial_port.isOpen() || !allow_send ||                   // Serial not ready
        ros::Time::now() - last_config_req < ros::Duration(0.5))  // Timeout waiting for response not reached
      return;
    publishLowLevelConfig(PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ);
    last_config_req = ros::Time::now();
    tries_left--;
    ROS_WARN_STREAM_COND(
        !tries_left, "Didn't received a config packet from LowLevel in time. Is your LowLevel firmware up-to-date?");
  };
} configTracker;

void publishActuatorsTimerTask(const ros::TimerEvent &timer_event) {
  updateEmergencyBits();
  publishActuators();
  publishStatus();
  //do not publish config, LL do not support it at the time
  //configTracker.check();
}

bool setMowEnabled(mower_msgs::MowerControlSrvRequest &req, mower_msgs::MowerControlSrvResponse &res) {
  if (req.mow_enabled != mower_enabled) {
    ROS_INFO_STREAM("[mower_comms] setMowEnabled(en=" << static_cast<unsigned>(req.mow_enabled)
                                                      << ", dir=" << static_cast<unsigned>(req.mow_direction) << ")");
  }
  mower_enabled = req.mow_enabled;
  mower_direction = req.mow_direction;
  if (mower_enabled && !isEmergency()) {
    speed_mow = req.mow_direction ? req.mow_power : -req.mow_power;
  } else {
    speed_mow = 0;
  }
  //    ROS_INFO_STREAM("[mower_comms] Setting mow enabled to " << speed_mow);
  return true;
}

bool setEmergencyMode(mower_msgs::EmergencyModeSrvRequest &req, mower_msgs::EmergencyModeSrvResponse &res) {
  uint8_t emergency_bit = req.emergency_bit;
  uint8_t emergency_code = 1 << emergency_bit;
  if (emergency_bit == mower_msgs::EmergencyModeSrvRequest::EMERGENCY_ALL) {
    ROS_WARN_STREAM("[mower_comms] Emergency ALL action");
    for (uint8_t bit = 0; bit < 8; bit++) {
      req.emergency_bit = bit;
      setEmergencyMode(req, res);
    }
    req.emergency_bit = mower_msgs::EmergencyModeSrvRequest::EMERGENCY_ALL;
    return true;
  }
  if (req.set_reset) {
    if (emergency_high_level_bits & emergency_code && emergency_high_level_end[emergency_bit].isZero() &&
        !req.duration.isZero()) {
      // active high level emergency has infinite duration, do not override it
      ROS_WARN_STREAM("[mower_comms] Do not overide previous inifinite duration emergency with reason [" << req.reason
                                                                                                         << "]");
    } else {
      ROS_ERROR_STREAM("[mower_comms] Setting emergency bit " << (int)emergency_bit << " with reason [" << req.reason
                                                              << "] duration " << req.duration.toSec());
      emergency_high_level_reasons[emergency_bit] = req.reason;
      emergency_high_level_end[emergency_bit] =
          req.duration.isZero() ? ros::Time::ZERO : ros::Time::now() + req.duration;
    }
    emergency_high_level_bits |= emergency_code;
  } else {
    if (emergency_high_level_bits & emergency_code) {
      ROS_WARN_STREAM("[mower_comms] Clear emergency bit " << (int)emergency_bit << " with reason [" << req.reason
                                                           << "]");
      emergency_high_level_reasons[emergency_bit] = req.reason;
      emergency_high_level_end[emergency_bit] = ros::Time::ZERO;
    }
    emergency_high_level_bits &= ~emergency_code;
  }

  // Set the high level emergency instantly. Low level value will be set on next update.
  // High level emergency already set with unlimited duration

  publishActuators();
  return true;
}

void highLevelStatusReceived(const mower_msgs::HighLevelStatus::ConstPtr &msg) {
  // ROS_INFO_STREAM("[mower_comms] High level status received: "<< msg->state_name << "/" << msg->sub_state_name);
  struct ll_high_level_state hl_state = {.type = PACKET_ID_LL_HIGH_LEVEL_STATE,
                                         .current_mode = msg->state,
                                         .gps_quality = static_cast<uint8_t>(msg->gps_quality_percent * 100.0)};

  sendLLMessage((uint8_t *)&hl_state, sizeof(struct ll_high_level_state));
}

void onCmdVelReceived(const geometry_msgs::Twist::ConstPtr &msg) {
  // TODO: update this to rad/s values and implement xESC speed control
  // ROS_INFO_STREAM("[mower_comms] Got Twist: "<< +msg->linear.x << " " << +msg->angular.z);
  last_cmd_twist = *msg;
  last_cmd_twist_time = ros::Time::now();
}

void onRearStateReceived(const hoverboard_driver::HoverboardStateStamped::ConstPtr &msg) {
  // ROS_INFO_STREAM("[mower_comms] Got rear driver state: "<< +msg->state.connection_state);
  last_rear_status = *msg;
}

void onFrontStateReceived(const hoverboard_driver::HoverboardStateStamped::ConstPtr &msg) {
  // ROS_INFO_STREAM("[mower_comms] Got front driver state: "<< +msg->state.connection_state);
  last_front_status = *msg;
}

void handleLowLevelUIEvent(struct ll_ui_event *ui_event) {
  ROS_INFO_STREAM("[mower_comms] Got UI button with code:" << +ui_event->button_id
                                                           << " and duration: " << +ui_event->press_duration);

  mower_msgs::HighLevelControlSrv srv;

  switch (ui_event->button_id) {
    case 2:
      // Home
      srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_HOME;
      break;
    case 3:
      // Play
      srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_START;
      break;
    case 4:
      // S1
      srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_S1;
      break;
    case 5:
      // S2
      if (ui_event->press_duration == 2) {
        srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_DELETE_MAPS;
      } else {
        srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_S2;
      }
      break;
    case 6:
      // LOCK
      if (ui_event->press_duration == 2) {
        // very long press on lock
        srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_RESET_EMERGENCY;
      }
      break;
    default:
      // Return, don't call the service.
      return;
  }

  if (!highLevelClient.call(srv)) {
    ROS_ERROR_STREAM("[mower_comms] Error calling high level control service");
  }
}

/**
 * @brief getNewSetChanged return t_new and checks if the value changed in comparison to t_cur.
 * t_new can't be a reference because the same function is also used for packed structures.
 * @param t_cur source value
 * @param t_new reference
 * @return &bool get set to true if t_cur and t_new differ, otherwise changed doesn't get touched
 */
template <typename T>
T getNewSetChanged(const T t_cur, const T t_new, bool &changed) {
  bool equal;
  if (std::is_floating_point<T>::value)
    equal = fabs(t_cur - t_new) < std::numeric_limits<T>::epsilon();
  else
    equal = t_cur == t_new;

  if (!equal) changed = true;

  //ROS_INFO_STREAM("DEBUG mower_comms comp. member: cur " << t_cur << " ?= " << t_new << " == equal " << equal << ", changed " << changed);

  return t_new;
}

/**
 * Handle config packet on receive from LL (LL->HL config packet response)
 */
void handleLowLevelConfig(const uint8_t *buffer, const size_t size) {
  // This is a flexible length packet where the size may vary when ll_high_level_config struct got enhanced only on one
  // side. If payload size is larger than our struct size, ensure that we only copy those we know of = our struct size.
  // If payload size is smaller than our struct size, copy only the payload we got, but ensure that the unsent member(s)
  // have reasonable defaults.
  size_t payload_size = std::min(sizeof(ll_high_level_config), size - 3);  // exclude type & crc

  // Copy payload to separated ll_config
  memcpy(&llhl_config, buffer + 1, payload_size);

  // Let's be verbose for easier follow-up
  ROS_INFO(
      "[mower_comms] Received ll_high_level_config packet %#04x\n"
      "\t options{dfp_is_5v=%d, background_sounds=%d, ignore_charging_current=%d},\n"
      "\t v_charge_cutoff=%f, i_charge_cutoff=%f,\n"
      "\t v_battery_cutoff=%f, v_battery_empty=%f, v_battery_full=%f,\n"
      "\t lift_period=%d, tilt_period=%d,\n"
      "\t shutdown_esc_max_pitch=%d,\n"
      "\t language=\"%.2s\", volume=%d\n"
      "\t hall_configs=\"%s\"",
      *buffer, (int)llhl_config.options.dfp_is_5v, (int)llhl_config.options.background_sounds,
      (int)llhl_config.options.ignore_charging_current, llhl_config.v_charge_cutoff, llhl_config.i_charge_cutoff,
      llhl_config.v_battery_cutoff, llhl_config.v_battery_empty, llhl_config.v_battery_full, llhl_config.lift_period,
      llhl_config.tilt_period, llhl_config.shutdown_esc_max_pitch, llhl_config.language, llhl_config.volume,
      getHallConfigsString(llhl_config.hall_configs, MAX_HALL_INPUTS).c_str());

  // Inform config packet tracker about the response
  configTracker.ackResponse();

  // Copy received config values from LL to mower_logic's related dynamic reconfigure variables and
  // decide if mower_logic's dynamic reconfigure need to be updated with probably changed values
  bool dirty = false;
  // clang-format off
  mower_logic_config.cu_rain_threshold = getNewSetChanged<int>(mower_logic_config.cu_rain_threshold, llhl_config.rain_threshold, dirty);
  mower_logic_config.charge_critical_high_voltage = getNewSetChanged<double>(mower_logic_config.charge_critical_high_voltage, llhl_config.v_charge_cutoff, dirty);
  mower_logic_config.charge_critical_high_current = getNewSetChanged<double>(mower_logic_config.charge_critical_high_current, llhl_config.i_charge_cutoff, dirty);
  mower_logic_config.battery_critical_high_voltage = getNewSetChanged<double>(mower_logic_config.battery_critical_high_voltage, llhl_config.v_battery_cutoff, dirty);
  mower_logic_config.battery_empty_voltage = getNewSetChanged<double>(mower_logic_config.battery_empty_voltage, llhl_config.v_battery_empty, dirty);
  mower_logic_config.battery_full_voltage = getNewSetChanged<double>(mower_logic_config.battery_full_voltage, llhl_config.v_battery_full, dirty);
  mower_logic_config.emergency_lift_period = getNewSetChanged<int>(mower_logic_config.emergency_lift_period, llhl_config.lift_period, dirty);
  mower_logic_config.emergency_tilt_period = getNewSetChanged<int>(mower_logic_config.emergency_tilt_period, llhl_config.tilt_period, dirty);
  // clang-format on

  if (dirty) reconfigClient->setConfiguration(mower_logic_config);
}

void handleLowLevelStatus(struct ll_status *status) {
  std::unique_lock<std::mutex> lk(ll_status_mutex);
  bool prev_esc_enabled = last_ll_status.status_bitmask & (1 << STATUS_ESC_ENABLED_BIT);
  bool new_esc_enabled = status->status_bitmask & (1 << STATUS_ESC_ENABLED_BIT);
  if (!prev_esc_enabled && new_esc_enabled) {
    last_ll_status_esc_enabled = ros::Time::now();
  }
  last_ll_status = *status;
  last_ll_status_time = ros::Time::now();
}

void handleLowLevelIMU(struct ll_imu *imu) {
  // imu_msg.dt = imu->dt_millis;
  sensor_mag_msg.header.stamp = ros::Time::now();
  sensor_mag_msg.header.seq++;
  sensor_mag_msg.header.frame_id = "base_link";
  sensor_mag_msg.magnetic_field.x = imu->mag_uT[0] / 1000.0;
  sensor_mag_msg.magnetic_field.y = imu->mag_uT[1] / 1000.0;
  sensor_mag_msg.magnetic_field.z = imu->mag_uT[2] / 1000.0;

  sensor_imu_msg.header.stamp = ros::Time::now();
  sensor_imu_msg.header.seq++;
  sensor_imu_msg.header.frame_id = "base_link";
  sensor_imu_msg.linear_acceleration.x = imu->acceleration_mss[imu_accel_idx[0]] * imu_accel_multiplier[0];
  sensor_imu_msg.linear_acceleration.y = imu->acceleration_mss[imu_accel_idx[1]] * imu_accel_multiplier[1];
  sensor_imu_msg.linear_acceleration.z = imu->acceleration_mss[imu_accel_idx[2]] * imu_accel_multiplier[2];
  sensor_imu_msg.angular_velocity.x = imu->gyro_rads[imu_gyro_idx[0]] * imu_gyro_multiplier[0];
  sensor_imu_msg.angular_velocity.y = imu->gyro_rads[imu_gyro_idx[1]] * imu_gyro_multiplier[1];
  sensor_imu_msg.angular_velocity.z = imu->gyro_rads[imu_gyro_idx[2]] * imu_gyro_multiplier[2];

  sensor_imu_pub.publish(sensor_imu_msg);
  sensor_mag_pub.publish(sensor_mag_msg);
}

int parseAxes(ros::NodeHandle &paramNh, double *axes_multiplier, int *axes_idx, const std::string &axes_param) {
  std::string axes = "+x+y+z";
  if (!paramNh.getParam(axes_param, axes)) {
    ROS_ERROR_STREAM("[mower_comms] IMU axes ("
                     << axes_param
                     << " param) orientation is not specified. Example \"+x-y-z\" for upside-down configuration");
    return 0;
  }
  if (axes.length() != 6) {
    ROS_ERROR_STREAM("[mower_comms] IMU axes ("
                     << axes_param
                     << " param) orientation foramt is incorrect. Example \"+x-y-z\" for upside-down configuration");
    return 0;
  }
  axes_multiplier[0] = axes.at(0) == '+' ? 1.0 : axes.at(0) == '-' ? -1.0 : 0;
  axes_idx[0] = axes.at(1) == 'x' ? 0 : axes.at(1) == 'y' ? 1 : axes.at(1) == 'z' ? 2 : -1;
  axes_multiplier[1] = axes.at(2) == '+' ? 1.0 : axes.at(2) == '-' ? -1.0 : 0;
  axes_idx[1] = axes.at(3) == 'x' ? 0 : axes.at(3) == 'y' ? 1 : axes.at(3) == 'z' ? 2 : -1;
  axes_multiplier[2] = axes.at(4) == '+' ? 1.0 : axes.at(4) == '-' ? -1.0 : 0;
  axes_idx[2] = axes.at(5) == 'x' ? 0 : axes.at(5) == 'y' ? 1 : axes.at(5) == 'z' ? 2 : -1;
  if (axes_idx[0] == -1 || axes_idx[1] == -1 || axes_idx[2] == -1 || axes_multiplier[0] == 0 ||
      axes_multiplier[1] == 0 || axes_multiplier[2] == 0) {
    ROS_ERROR_STREAM("[mower_comms] IMU axes ("
                     << axes_param
                     << " param) orientation foramt is incorrect. Example \"+x-y-z\" for upside-down configuration");
    return 0;
  }
  return 1;
}

void reconfigCB(const mower_logic::MowerLogicConfig &config) {
  ROS_INFO_STREAM("[mower_comms] received new mower_logic config");

  mower_logic_config = config;

  // Copy changed mower_config's values to the related llhl_config values and
  // decide if LL need to be informed with a new config packet
  bool dirty = false;

  // clang-format off
  llhl_config.rain_threshold = getNewSetChanged<int>(llhl_config.rain_threshold, mower_logic_config.cu_rain_threshold, dirty);
  llhl_config.v_charge_cutoff = getNewSetChanged<double>(llhl_config.v_charge_cutoff, mower_logic_config.charge_critical_high_voltage, dirty);
  llhl_config.i_charge_cutoff = getNewSetChanged<double>(llhl_config.i_charge_cutoff, mower_logic_config.charge_critical_high_current, dirty);
  llhl_config.v_battery_cutoff = getNewSetChanged<double>(llhl_config.v_battery_cutoff, mower_logic_config.battery_critical_high_voltage, dirty);
  llhl_config.v_battery_empty = getNewSetChanged<double>(llhl_config.v_battery_empty, mower_logic_config.battery_empty_voltage, dirty);
  llhl_config.v_battery_full = getNewSetChanged<double>(llhl_config.v_battery_full, mower_logic_config.battery_full_voltage, dirty);
  llhl_config.lift_period = getNewSetChanged<int>(llhl_config.lift_period, mower_logic_config.emergency_lift_period, dirty);
  llhl_config.tilt_period = getNewSetChanged<int>(llhl_config.tilt_period, mower_logic_config.emergency_tilt_period, dirty);
  // clang-format on

  // Parse emergency_input_config and set hall_configs
  char *token = strtok(strdup(mower_logic_config.emergency_input_config.c_str()), ",");
  bool low_active;
  unsigned int hall_idx = 0;
  while (token != NULL) {
    low_active = false;
    while (*token != 0) {
      switch (std::toupper(*token)) {
        case '!': low_active = true; break;
        case 'I': llhl_config.hall_configs[hall_idx] = {HallMode::OFF, low_active}; break;
        case 'L': llhl_config.hall_configs[hall_idx] = {HallMode::LIFT_TILT, low_active}; break;
        case 'S': llhl_config.hall_configs[hall_idx] = {HallMode::STOP, low_active}; break;
        case 'U': llhl_config.hall_configs[hall_idx] = {HallMode::UNDEFINED, low_active}; break;
        default: break;
      }
      token++;
    }
    token = strtok(NULL, ",");
    hall_idx++;
  }

  if (dirty) configTracker.setDirty();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mower_comms");

  sensor_mag_msg.header.seq = 0;
  sensor_imu_msg.header.seq = 0;

  ros::NodeHandle n;
  ros::NodeHandle paramNh("~");
  ros::NodeHandle mowerParamNh("~/mower_xesc");

  highLevelClient = n.serviceClient<mower_msgs::HighLevelControlSrv>("mower_service/high_level_control");

  mower_logic_config = mower_logic::MowerLogicConfig::__getDefault__();
  reconfigClient = new dynamic_reconfigure::Client<mower_logic::MowerLogicConfig>("/mower_logic", reconfigCB);

  std::string ll_serial_port_name;
  if (!paramNh.getParam("ll_serial_port", ll_serial_port_name)) {
    ROS_ERROR_STREAM("[mower_comms] Error getting low level serial port parameter. Quitting.");
    return 1;
  }

  if (!paramNh.getParam("wheel_radius_m", wheel_radius_m)) {
    ROS_ERROR_STREAM("[mower_comms] Wheel radius must be specified for odometry. Quitting.");
    return 1;
  }

  if (!paramNh.getParam("wheel_separation_m", wheel_separation_m)) {
    ROS_ERROR_STREAM("[mower_comms] Wheel separation must be specified for odometry. Quitting.");
    return 1;
  }

  if (!paramNh.getParam("mower_esc_enabled", mower_esc_enabled)) {
    ROS_ERROR_STREAM("[mower_comms] Mower ESC enabled parameter is not specified. Quitting.");
    return 1;
  }

  if (paramNh.param("cmd_vel_timout", cmd_vel_timout, 1.0)) {
    ROS_INFO_STREAM("[mower_comms] Configured cmd_vel_timout: " << cmd_vel_timout);
  }

  if (!parseAxes(paramNh, imu_accel_multiplier, imu_accel_idx, "imu_accel_axes")) {
    return 1;
  }

  if (!parseAxes(paramNh, imu_gyro_multiplier, imu_gyro_idx, "imu_gyro_axes")) {
    return 1;
  }

  ROS_INFO_STREAM("[mower_comms] Wheel radius [m]: " << wheel_radius_m << " , separation [m]: " << wheel_separation_m);

  last_cmd_twist.linear.x = 0;
  last_cmd_twist.angular.z = 0;
  speed_mow = 0;

  // Some generic settings from param server (non- dynamic)
  llhl_config.options.ignore_charging_current =
      paramNh.param("/mower_logic/ignore_charging_current", false) ? OptionState::ON : OptionState::OFF;
  llhl_config.options.dfp_is_5v = paramNh.param("dfp_is_5v", false) ? OptionState::ON : OptionState::OFF;
  llhl_config.volume = paramNh.param("volume", -1);
  llhl_config.options.background_sounds =
      paramNh.param("background_sounds", false) ? OptionState::ON : OptionState::OFF;
  // ISO-639-1 (2 char) language code
  strncpy(llhl_config.language, paramNh.param<std::string>("language", "en").c_str(), 2);

  // Setup XESC interfaces
  if (mowerParamNh.hasParam("xesc_type") && mower_esc_enabled) {
    mow_xesc_interface = new xesc_driver::XescDriver(n, mowerParamNh);
  } else {
    mow_xesc_interface = nullptr;
  }

  status_pub = n.advertise<mower_msgs::Status>("mower/status", 1);
  wheel_tick_pub = n.advertise<xbot_msgs::WheelTick>("mower/wheel_ticks", 1);
  sensor_imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
  sensor_mag_pub = n.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
  cmd_vel_safe_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_safe", 1);

  ros::ServiceServer mow_service = n.advertiseService("mower_service/mow_enabled", setMowEnabled);
  ros::ServiceServer emergency_service = n.advertiseService("mower_service/emergency", setEmergencyMode);
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 0, onCmdVelReceived, ros::TransportHints().tcpNoDelay(true));
  ros::Subscriber high_level_status_sub = n.subscribe("/mower_logic/current_state", 0, highLevelStatusReceived);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.02), publishActuatorsTimerTask);

  ros::Subscriber rear_state_sub =
      n.subscribe("/rear/hoverboard_driver/state", 0, onRearStateReceived, ros::TransportHints().tcpNoDelay(true));
  ros::Subscriber front_state_sub =
      n.subscribe("/front/hoverboard_driver/state", 0, onFrontStateReceived, ros::TransportHints().tcpNoDelay(true));

  size_t buflen = 1000;
  uint8_t buffer[buflen];
  uint8_t buffer_decoded[buflen];
  size_t read = 0;
  // don't change, we need to wait for arduino to boot before actually sending stuff
  ros::Duration retryDelay(5, 0);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while (ros::ok()) {
    if (!serial_port.isOpen()) {
      ROS_INFO_STREAM("[mower_comms] connecting serial interface: " << ll_serial_port_name);
      allow_send = false;
      try {
        serial_port.setPort(ll_serial_port_name);
        serial_port.setBaudrate(115200);
        auto to = serial::Timeout::simpleTimeout(100);
        serial_port.setTimeout(to);
        serial_port.open();

        // wait for controller to boot
        retryDelay.sleep();
        // this will only be set if no error was set

        allow_send = true;
      } catch (std::exception &e) {
        retryDelay.sleep();
        ROS_ERROR_STREAM("[mower_comms] Error during reconnect.");
      }
    }
    size_t bytes_read = 0;
    try {
      bytes_read = serial_port.read(buffer + read, 1);
    } catch (std::exception &e) {
      ROS_ERROR_STREAM("[mower_comms] Error reading serial_port. Closing Connection.");
      serial_port.close();
      retryDelay.sleep();
    }
    if (read + bytes_read >= buflen) {
      read = 0;
      bytes_read = 0;
      ROS_ERROR_STREAM("[mower_comms] Prevented buffer overflow. There is a problem with the serial comms.");
    }
    if (bytes_read) {
      if (buffer[read] == 0) {
        // end of packet found
        size_t data_size = cobs.decode(buffer, read, buffer_decoded);

        // first, check the CRC
        if (data_size < 3) {
          // We don't even have one byte of data
          // (type + crc = 3 bytes already)
          ROS_INFO_STREAM("[mower_comms] Got empty packet from Low Level Board");
        } else {
          // We have at least 1 byte of data, check the CRC
          crc.reset();
          // We start at the second byte (ignore the type) and process (data_size- byte for type - 2 bytes for CRC)
          // bytes.
          crc.process_bytes(buffer_decoded, data_size - 2);
          uint16_t checksum = crc.checksum();
          uint16_t received_checksum = *(uint16_t *)(buffer_decoded + data_size - 2);
          if (checksum == received_checksum) {
            // Packet checksum is OK, process it
            switch (buffer_decoded[0]) {
              case PACKET_ID_LL_STATUS:
                if (data_size == sizeof(struct ll_status)) {
                  handleLowLevelStatus((struct ll_status *)buffer_decoded);
                } else {
                  ROS_WARN_STREAM(
                      "[mower_comms] Low Level Board sent a valid packet with the wrong size. Type was STATUS");
                }
                break;
              case PACKET_ID_LL_IMU:
                if (data_size == sizeof(struct ll_imu)) {
                  handleLowLevelIMU((struct ll_imu *)buffer_decoded);
                } else {
                  ROS_WARN_STREAM(
                      "[mower_comms] Low Level Board sent a valid packet with the wrong size. Type was IMU");
                }
                break;
              case PACKET_ID_LL_UI_EVENT:
                if (data_size == sizeof(struct ll_ui_event)) {
                  handleLowLevelUIEvent((struct ll_ui_event *)buffer_decoded);
                } else {
                  ROS_WARN_STREAM(
                      "[mower_comms] Low Level Board sent a valid packet with the wrong size. Type was UI_EVENT");
                }
                break;
              case PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ:
              case PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP:
                handleLowLevelConfig(buffer_decoded, data_size);
                break;
              default: ROS_WARN_STREAM("[mower_comms] Got unknown packet from Low Level Board"); break;
            }
          } else {
            ROS_WARN_STREAM("[mower_comms] Got invalid checksum from Low Level Board");
          }
        }

        read = 0;
      } else {
        read += bytes_read;
      }
    }
  }

  spinner.stop();

  if (mow_xesc_interface) {
    mow_xesc_interface->setDutyCycle(0.0);
    mow_xesc_interface->stop();
  }
  last_cmd_twist.linear.x = 0;
  last_cmd_twist.angular.z = 0;
  publishActuators();
  // left_xesc_interface->stop();
  // right_xesc_interface->stop();

  if (mow_xesc_interface) {
    delete mow_xesc_interface;
  }

  return 0;
}
