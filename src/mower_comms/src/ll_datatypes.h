// Created by Clemens Elflein on 3/07/22.
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

#ifndef _DATATYPES_H
#define _DATATYPES_H

#include <stdint.h>

#define PACKET_ID_LL_STATUS 1
#define PACKET_ID_LL_IMU 2
#define PACKET_ID_LL_UI_EVENT 3
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

#define EMERGENCY_BUTTON1_BIT 1 //primary emergency button
#define EMERGENCY_BUTTON2_BIT 2
#define EMERGENCY_LIFT1_BIT 3
#define EMERGENCY_LIFT2_BIT 4
#define EMERGENCY_ROS_TIMEOUT 5
#define EMERGENCY_HIGH_LEVEL 6

// motor status bits
#define MOTOR_STATUS_DISABLED               0
#define MOTOR_STATUS_BAD_CTRL_MODE          1
#define MOTOR_STATUS_LEFT_MOTOR_ERR         2
#define MOTOR_STATUS_RIGHT_MOTOR_ERR        3

#define MOTOR_STATUS_PCB_TEMP_WARN          4
#define MOTOR_STATUS_PCB_TEMP_ERR           5
#define MOTOR_STATUS_LEFT_MOTOR_TEMP_ERR    6
#define MOTOR_STATUS_RIGHT_MOTOR_TEMP_ERR   7

#define MOTOR_STATUS_CONN_TIMEOUT           8
#define MOTOR_STATUS_ADC_TIMEOUT            9
#define MOTOR_STATUS_GEN_TIMEOUT            10

#define MOTOR_STATUS_BATTERY_DEAD           12
#define MOTOR_STATUS_BATTERY_L1             13
#define MOTOR_STATUS_BATTERY_L2             14

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
    // Bit 5: don't care
    // Bit 6: don't care
    // Bit 7: don't care
    uint8_t status_bitmask;
    // USS range in m
    float uss_ranges_m[5];
    // USS measurement age in ms (no more than UINT16_MAX)
    uint16_t uss_age_ms[5];
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
    float v_system;
    // Charge current
    float battery_current;
    uint8_t batt_percentage;
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
    //high level emergency bits (e.g. navigation error, ...)
    uint8_t emergency_high_level;
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct ll_ui_event {
    // Type of this message. Has to be PACKET_ID_LL_UI_EVENT
    uint8_t type;
    uint8_t button_id;
    uint8_t press_duration;   // 0 for single press, 1 for long, 2 for very long press
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct ll_high_level_state {
    // Type of this message. Has to be PACKET_ID_LL_HIGH_LEVEL_STATE
    uint8_t type;
    uint8_t current_mode; // see HighLevelMode
    uint8_t gps_quality;   // GPS quality in percent (0-100)
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct ll_motor_state {
    // Type of this message. Has to be PACKET_ID_LL_HIGH_LEVEL_STATE
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

#endif