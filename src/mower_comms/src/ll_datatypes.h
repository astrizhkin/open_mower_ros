// Created by Clemens Elflein on 3/07/22.
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

#ifndef _DATATYPES_H
#define _DATATYPES_H

#include <stdint.h>

#define PACKET_ID_LL_STATUS 1
#define PACKET_ID_LL_IMU 2
#define PACKET_ID_LL_UI_EVENT 3
#define PACKET_ID_LL_HIGH_LEVEL_CONFIG_GET 0x10  // ll_high_level_config read value result
#define PACKET_ID_LL_HIGH_LEVEL_CONFIG_SET 0x11  // ll_high_level_config set value result
#define PACKET_ID_LL_HIGH_LEVEL_CONFIG_ERR 0x12  // ll_high_level_config invalid address
#define PACKET_ID_LL_HEARTBEAT 0x42
#define PACKET_ID_LL_HIGH_LEVEL_STATE 0x43
#define PACKET_ID_LL_MOTOR_STATE 0x44

#define STATUS_INIT_BIT 0
#define STATUS_RASPI_POWER_BIT 1
#define STATUS_CHARGING_BIT 2
#define STATUS_ESC_ENABLED_BIT 3
#define STATUS_RAIN_BIT 4
#define STATUS_USS_TIMEOUT_BIT 5
#define STATUS_IMU_TIMEOUT_BIT 6
#define STATUS_BATTERY_EMPTY_BIT 7
#define STATUS_BMS_TIMEOUT_BIT 8

#define EMERGENCY_CONTACT1_BIT 1 //primary emergency button
#define EMERGENCY_CONTACT2_BIT 2
#define EMERGENCY_CONTACT3_BIT 3
#define EMERGENCY_CONTACT4_BIT 4
#define EMERGENCY_ROS_TIMEOUT 5
#define EMERGENCY_HIGH_LEVEL 6

#define POWER_REQUEST_PI                            0
#define POWER_REQUEST_MOTOR                         1
#define POWER_REQUEST_LED                           2
#define POWER_REQUEST_RESERVED                      3

#define POWER_REQUEST_BITS_OFF                      0
#define POWER_REQUEST_BITS_ON                       1
#define POWER_REQUEST_BITS_LOW_FREQ                 2
#define POWER_REQUEST_BITS_HIGH_FREQ                3

#define USS_COUNT 5
#define CONTACT_COUNT 4

#pragma pack(push, 1)
struct ll_status {
    // Type of this message. Has to be PACKET_ID_LL_STATUS.
    uint8_t type;
    // Bitmask for rain, sound, powers etc
    // Bit 0: Initialized (i.e. setup() was a success). If this is 0, all other bits are meaningless.
    // Bit 1: Raspberry Power
    // Bit 2: Charging enabled
    // Bit 3: ESC power enabled
    // Bit 4: Rain detected
    // Bit 5-15: don't care

    uint16_t status_bitmask;
    // USS range in m
    uint8_t uss_ranges_cm[USS_COUNT];
    // USS measurement age in ms (no more than UINT16_MAX)
    uint16_t uss_age_ms[USS_COUNT];
    //contact active after timeout
    uint8_t contacts;
    // Emergency bitmask:
    // Bit 0: Emergency latch
    // Bit 1: Emergency 0 active
    // Bit 2: Emergency 1 active
    // Bit 3: Emergency 2 active
    // Bit 4: Emergency 3 active
    // Bit 5: Emergency USS tiemout
    // Bit 6: Emergency IMU tiemout
    // Bit 7: Battery empty
    uint8_t emergency_bitmask;
    // Charge voltage
    float v_charge;
    // System voltage
    float v_battery;
    // Charge current
    float battery_current;
    uint8_t battery_soc;
    float battery_temperature;
    float balancer_temperature;
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct ll_imu {
  // Type of this message. Has to be PACKET_ID_LL_IMU.
  uint8_t type;
  // Time since last message in milliseconds.
  uint16_t dt_millis;
  // Acceleration[m^s2], Gyro[rad/s] and magnetic field[uT]
  float acceleration_mss[3];
  float gyro_rads[3];
  float mag_uT[3];
  uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct ll_heartbeat {
    // Type of this message. Has to be PACKET_ID_LL_HEARTBEAT.
    uint8_t type;
    // 4 x 2 bits per power output (see POWER_REQUEST_... defines)
    uint8_t power_request;
    //high level emergency bits (e.g. navigation error, ...)
    uint8_t emergency_high_level;
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

//#pragma pack(push, 1)
//struct ll_ui_event {
//  // Type of this message. Has to be PACKET_ID_LL_UI_EVENT
//  uint8_t type;
//  uint8_t button_id;
//  uint8_t press_duration;  // 0 for single press, 1 for long, 2 for very long press
//  uint16_t crc;
//} __attribute__((packed));
//#pragma pack(pop)

#pragma pack(push, 1)
struct ll_high_level_state {
  // Type of this message. Has to be PACKET_ID_LL_HIGH_LEVEL_STATE
  uint8_t type;
  uint8_t current_mode;  // see HighLevelMode
  uint8_t gps_quality;   // GPS quality in percent (0-100)
  uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct ll_motor_state {
    // Type of this message. Has to be PACKET_ID_LL_MOTOR_STATE
    uint8_t type;
    //uint8_t motor_id;

    //rad/s
    //float cmd[5];
    //rad/s
    //float speed_meas[5];
    //rad
    //float wheel_cnt[5];
    //float curr_meas[5];
    //float motor_temp[5];
    //float boardTemp[3];
    uint16_t status[3]; //See motor status bits
    uint8_t status_age_s[3];
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

typedef enum ConfigCommand {
    CONFIGURATION_LOAD = 1,
    CONFIGURATION_SAVE = 2
} ConfigCommand;

typedef enum ConfigAddress {
  //control group (10)
  EEPROM_STATUS = 0, //bool
  COMMAND = 1, //bool

  //charge group (20)
  CHARGE_START_SOC = 10, //int percent, 0 - disable
  CHARGE_START_VOLTAGE = 11,//float voltage, 0 - disable
  CHARGE_STOP_SOC = 12,  //int percent, 0 - disable
  CHARGE_STOP_VOLTAGE = 13,//float voltage, 0 - disable
  CHARGE_STOP_CURRENT = 14,//float current, 0 - disable
  CHARGE_MAX_CURRENT = 15,//float current, 0 - disable
  CHARGER_MAX_VOLTAGE = 16,//float voltage, 0 - disable
  CHARGER_MIN_VOLTAGE = 17,//float voltage, 0 - disable
  CHARGE_MIN_BATTERY_TEMPERATURE = 18, //float temparature, 0 - disable
  CHARGE_MAX_BATTERY_TEMPERATURE = 19,//float temparature, 0 - disable
  CHARGE_STOP_BALANCER_TEMPERATURE = 20,//float temparature, 0 - disable

  //battery group (10)
  BATTERY_EMPTY_VOLTAGE = 30,//float voltage, 0 - disable
  BATTERY_FULL_VOLTAGE = 31,//float voltage, 0 - disable
  BATTERY_LOW_WARNING_SOC = 32,//int percent, 0 - disable
  BATTERY_LOW_WARNING_VOLTAGE = 33,//float voltage, 0 - disable
  BATTERY_SHUTDOWN_SOC = 34,//int percent, 0 - disable
  BATTERY_SHUTDOWN_VOLTAGE = 35,//float voltage, 0 - disable

  //contact group (4x8)
  CONTACT_MODE = 40,//ContactMode, address2 is requred
  CONTACT_ACTIVE_LOW = 41,//bool, address2 is requred
  CONTACT_TIMEOUT = 42,//bool, address2 is requred

  //uss group (4x8)
  USS_ACTIVE = 40+32,//bool, address2 is requred

  //end
  END = 30+32+32
} ConfigAddress;

typedef enum ContactMode {
  OFF = 0,
  MONITOR = 1,  // Contact output
  EMERGENCY_STOP = 2, // Emergeny bit and contact output
} ContactMode;

union ConfigValue {
//  double doubleValue;
  float floatValue;
  uint32_t uint32Value;
  int32_t int32Value;
  uint16_t uint16Value;
  int16_t int16Value;
  uint8_t uint8Value;
  int8_t int8Value;
  char charValue;
  bool boolValue;
};

// LL/HL config packet, bi-directional, flexible-length
#pragma pack(push, 1)
struct ll_high_level_config {
  // PACKET_ID_LL_HIGH_LEVEL_CONFIG_*
  uint8_t type;
  uint8_t address;
  uint8_t address2;//4 bits actually
  ConfigValue value;
  uint16_t crc;
  // clang-format on
} __attribute__((packed));
#pragma pack(pop)

#endif
