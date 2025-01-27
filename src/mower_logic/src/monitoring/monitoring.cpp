// Created by Clemens Elflein on 3/28/22.
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

#include "ros/ros.h"
#include "xbot_msgs/RobotState.h"
#include "xbot_msgs/AbsolutePose.h"
#include "xbot_msgs/SensorInfo.h"
#include "xbot_msgs/SensorDataDouble.h"
#include "mower_msgs/HighLevelStatus.h"
#include "mower_msgs/Status.h"

ros::Publisher state_pub;
xbot_msgs::RobotState state;

xbot_msgs::SensorInfo si_v_charge;
ros::Publisher si_v_charge_pub;
ros::Publisher v_charge_data_pub;

xbot_msgs::SensorInfo si_v_battery;
ros::Publisher si_v_battery_pub;
ros::Publisher v_battery_data_pub;

xbot_msgs::SensorInfo si_battery_current;
ros::Publisher si_battery_current_pub;
ros::Publisher battery_current_data_pub;

xbot_msgs::SensorInfo si_battery_soc;
ros::Publisher si_battery_soc_pub;
ros::Publisher battery_soc_data_pub;

xbot_msgs::SensorInfo si_rear_left_esc_temp;
ros::Publisher si_rear_left_esc_temp_pub;
ros::Publisher rear_left_esc_temp_data_pub;

xbot_msgs::SensorInfo si_rear_right_esc_temp;
ros::Publisher si_rear_right_esc_temp_pub;
ros::Publisher rear_right_esc_temp_data_pub;

xbot_msgs::SensorInfo si_front_left_esc_temp;
ros::Publisher si_front_left_esc_temp_pub;
ros::Publisher front_left_esc_temp_data_pub;

xbot_msgs::SensorInfo si_front_right_esc_temp;
ros::Publisher si_front_right_esc_temp_pub;
ros::Publisher front_right_esc_temp_data_pub;

xbot_msgs::SensorInfo si_mow_esc_temp;
ros::Publisher si_mow_esc_temp_pub;
ros::Publisher mow_esc_temp_data_pub;

xbot_msgs::SensorInfo si_mow_motor_temp;
ros::Publisher si_mow_motor_temp_pub;
ros::Publisher mow_motor_temp_data_pub;

xbot_msgs::SensorInfo si_mow_motor_current;
ros::Publisher si_mow_motor_current_pub;
ros::Publisher mow_motor_current_data_pub;

xbot_msgs::SensorInfo si_gps_accuracy;
ros::Publisher si_gps_accuracy_pub;
ros::Publisher gps_accuracy_data_pub;

ros::NodeHandle *n;

ros::Time last_status_update(0);
ros::Time last_pose_update(0);

void status(const mower_msgs::Status::ConstPtr &msg) {
    // Rate limit to 2Hz
    if((msg->stamp - last_status_update).toSec() < 0.5)
        return;
    last_status_update = msg->stamp;

    xbot_msgs::SensorDataDouble sensor_data;
    sensor_data.stamp = msg->stamp;

    sensor_data.data = msg->v_charge;
    v_charge_data_pub.publish(sensor_data);

    sensor_data.data = msg->v_battery;
    v_battery_data_pub.publish(sensor_data);

    sensor_data.data = msg->battery_current;
    battery_current_data_pub.publish(sensor_data);

    sensor_data.data = msg->battery_soc;
    battery_soc_data_pub.publish(sensor_data);

    sensor_data.data = msg->rear_left_esc_status.temperature_pcb;
    rear_left_esc_temp_data_pub.publish(sensor_data);

    sensor_data.data = msg->rear_right_esc_status.temperature_pcb;
    rear_right_esc_temp_data_pub.publish(sensor_data);

    sensor_data.data = msg->front_left_esc_status.temperature_pcb;
    front_left_esc_temp_data_pub.publish(sensor_data);

    sensor_data.data = msg->front_right_esc_status.temperature_pcb;
    front_right_esc_temp_data_pub.publish(sensor_data);

    sensor_data.data = msg->mow_esc_status.temperature_pcb;
    mow_esc_temp_data_pub.publish(sensor_data);

    sensor_data.data = msg->mow_esc_status.temperature_motor;
    mow_motor_temp_data_pub.publish(sensor_data);
 
    sensor_data.data = msg->mow_esc_status.current;
    mow_motor_current_data_pub.publish(sensor_data);
}

void high_level_status(const mower_msgs::HighLevelStatus::ConstPtr &msg) {
    state.gps_percentage = msg->gps_quality_percent;
    state.current_state = msg->state_name;
    state.current_sub_state = msg->sub_state_name;
    state.battery_percentage = msg->battery_percent;
    state.emergency = msg->emergency;
    state.is_charging = msg->is_charging;

    state_pub.publish(state);
}

void pose_received(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
    state.robot_pose = *msg;

    // Rate limit to 2Hz
    if((msg->header.stamp - last_pose_update).toSec() < 0.5)
        return;
    last_pose_update = msg->header.stamp;

    xbot_msgs::SensorDataDouble sensor_data;
    sensor_data.stamp = msg->header.stamp;
    sensor_data.data = msg->position_accuracy;
    gps_accuracy_data_pub.publish(sensor_data);
}

void registerSensors() {
    si_v_charge.sensor_id = "om_v_charge";
    si_v_charge.sensor_name = "V Charge";
    si_v_charge.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_v_charge.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_VOLTAGE;
    si_v_charge.unit = "V";
    si_v_charge_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_v_charge.sensor_id + "/info", 1, true);
    v_charge_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_v_charge.sensor_id + "/data",10);
    si_v_charge_pub.publish(si_v_charge);

    si_v_battery.sensor_id = "om_v_battery";
    si_v_battery.sensor_name = "V Battery";
    si_v_battery.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_v_battery.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_VOLTAGE;
    si_v_battery.unit = "V";
    si_v_battery_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_v_battery.sensor_id + "/info", 1, true);
    v_battery_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_v_battery.sensor_id + "/data",10);
    si_v_battery_pub.publish(si_v_battery);

    si_battery_current.sensor_id = "om_battery_current";
    si_battery_current.sensor_name = "Battery Current";
    si_battery_current.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_battery_current.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_CURRENT;
    si_battery_current.unit = "A";
    si_battery_current_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_battery_current.sensor_id + "/info", 1, true);
    battery_current_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_battery_current.sensor_id + "/data",10);
    si_battery_current_pub.publish(si_battery_current);

    si_battery_soc.sensor_id = "om_battery_soc";
    si_battery_soc.sensor_name = "Battery SOC";
    si_battery_soc.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_battery_soc.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_CURRENT;
    si_battery_soc.unit = "%";
    si_battery_soc_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_battery_soc.sensor_id + "/info", 1, true);
    battery_soc_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_battery_soc.sensor_id + "/data",10);
    si_battery_soc_pub.publish(si_battery_soc);

    si_rear_left_esc_temp.sensor_id = "om_rear_left_esc_temp";
    si_rear_left_esc_temp.sensor_name = "Rear Left ESC Temp";
    si_rear_left_esc_temp.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_rear_left_esc_temp.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE;
    si_rear_left_esc_temp.unit = "deg.C";
    si_rear_left_esc_temp_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_rear_left_esc_temp.sensor_id + "/info", 1, true);
    rear_left_esc_temp_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_rear_left_esc_temp.sensor_id + "/data",10);
    si_rear_left_esc_temp_pub.publish(si_rear_left_esc_temp);

    si_rear_right_esc_temp.sensor_id = "om_rear_right_esc_temp";
    si_rear_right_esc_temp.sensor_name = "Rear Right ESC Temp";
    si_rear_right_esc_temp.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_rear_right_esc_temp.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE;
    si_rear_right_esc_temp.unit = "deg.C";
    si_rear_right_esc_temp_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_rear_right_esc_temp.sensor_id + "/info", 1, true);
    rear_right_esc_temp_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_rear_right_esc_temp.sensor_id + "/data",10);
    si_rear_right_esc_temp_pub.publish(si_rear_right_esc_temp);

    si_front_left_esc_temp.sensor_id = "om_front_left_esc_temp";
    si_front_left_esc_temp.sensor_name = "Front Left ESC Temp";
    si_front_left_esc_temp.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_front_left_esc_temp.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE;
    si_front_left_esc_temp.unit = "deg.C";
    si_front_left_esc_temp_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_front_left_esc_temp.sensor_id + "/info", 1, true);
    front_left_esc_temp_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_front_left_esc_temp.sensor_id + "/data",10);
    si_front_left_esc_temp_pub.publish(si_front_left_esc_temp);

    si_front_right_esc_temp.sensor_id = "om_front_right_esc_temp";
    si_front_right_esc_temp.sensor_name = "Front Right ESC Temp";
    si_front_right_esc_temp.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_front_right_esc_temp.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE;
    si_front_right_esc_temp.unit = "deg.C";
    si_front_right_esc_temp_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_front_right_esc_temp.sensor_id + "/info", 1, true);
    front_right_esc_temp_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_front_right_esc_temp.sensor_id + "/data",10);
    si_front_right_esc_temp_pub.publish(si_front_right_esc_temp);

    si_mow_esc_temp.sensor_id = "om_mow_esc_temp";
    si_mow_esc_temp.sensor_name = "Mow ESC Temp";
    si_mow_esc_temp.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_mow_esc_temp.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE;
    si_mow_esc_temp.unit = "deg.C";
    si_mow_esc_temp_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_mow_esc_temp.sensor_id + "/info", 1, true);
    mow_esc_temp_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_mow_esc_temp.sensor_id + "/data",10);
    si_mow_esc_temp_pub.publish(si_mow_esc_temp);

    si_mow_motor_temp.sensor_id = "om_mow_motor_temp";
    si_mow_motor_temp.sensor_name = "Mow Motor Temp";
    si_mow_motor_temp.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_mow_motor_temp.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE;
    si_mow_motor_temp.unit = "deg.C";
    si_mow_motor_temp_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_mow_motor_temp.sensor_id + "/info", 1, true);
    mow_motor_temp_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_mow_motor_temp.sensor_id + "/data",10);
    si_mow_motor_temp_pub.publish(si_mow_motor_temp);
    
    si_mow_motor_current.sensor_id = "om_mow_motor_current";
    si_mow_motor_current.sensor_name = "Mow Motor Current";
    si_mow_motor_current.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_mow_motor_current.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_CURRENT;
    si_mow_motor_current.unit = "A";
    si_mow_motor_current_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_mow_motor_current.sensor_id + "/info", 1, true);
    mow_motor_current_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_mow_motor_current.sensor_id + "/data",10);
    si_mow_motor_current_pub.publish(si_mow_motor_current);

    si_gps_accuracy.sensor_id = "om_gps_accuracy";
    si_gps_accuracy.sensor_name = "GPS Accuracy";
    si_gps_accuracy.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_gps_accuracy.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_PERCENT;
    si_gps_accuracy.unit = "%";
    si_gps_accuracy_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_gps_accuracy.sensor_id + "/info", 1, true);
    gps_accuracy_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_gps_accuracy.sensor_id + "/data",10);
    si_gps_accuracy_pub.publish(si_gps_accuracy);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "monitoring");

    n = new ros::NodeHandle();

    registerSensors();

    ros::Subscriber pose_sub = n->subscribe("xbot_positioning/xb_pose", 10, pose_received);
    ros::Subscriber state_sub = n->subscribe("mower_logic/current_state", 10, high_level_status);
    ros::Subscriber status_sub = n->subscribe("mower/status", 10, status);

    state_pub = n->advertise<xbot_msgs::RobotState>("xbot_monitoring/robot_state", 10);

    ros::spin();

    return 0;
}
