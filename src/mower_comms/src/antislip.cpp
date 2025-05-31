#include "ros/ros.h"
#include <tf2/LinearMath/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include "nav_msgs/Odometry.h"
#include <list>

struct StampedValue {
  double value;
  double time;//ros time in seconds
};

std::list<StampedValue> imu_angular_value_series;
//std::list<StampedValue> pose_linear_value_series;
std::list<StampedValue> wheel_angular_value_series;
//std::list<StampedValue> wheel_linear_value_series;
double sum_imu_rotation;
//double sum_pose_motion;
double sum_wheel_rotation;
//double sum_wheel_motion;

ros::Publisher cmd_vel_pub;

bool angularSlip;
ros::Time angularSlipStarted;

//configuration
double antislipVel;
double angularAntislipTimeout;
double integrationTime;
double absoluteAngularDifference;
double relativeAngularDifference;

void cleanOldValues(std::list<StampedValue>& list, double &sum, double delete_up_to_time) {
  std::list<StampedValue>::iterator it = list.begin();
  for (; it != list.end(); ) {
    if(it->time >= delete_up_to_time) {
      break;
    }
    sum -= it->value;
    it = list.erase(it);  
  }
}

void addValue(std::list<StampedValue>& list, double &sum, double value, double time) {
  if(!list.empty()){
    StampedValue &last_value = list.back();

    double dt = time - last_value.time;
    double change_dt = value * dt;
    //ROS_INFO_STREAM("[antislip] add "<<name<<" val "<<value<<" dt "<<dt<<" res"<<change_dt);
    StampedValue new_value = {.value = change_dt, .time = time};
    list.push_back(new_value);
    sum += change_dt;
  }else{
    StampedValue new_value = {.value = 0, .time = time};
    list.push_back(new_value);
  }
  
  cleanOldValues(list, sum, time - integrationTime);
}

double sum(std::list<StampedValue>& list) {
  double sum = 0;
  std::list<StampedValue>::iterator it = list.begin();
  for (; it != list.end(); ++it) {
    sum+=it->value;
  }
  return sum;
}

void onImu(const sensor_msgs::Imu::ConstPtr &msg) {
  //tf2::Vector3 imu_accel(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
  //tf2::Vector3 imu_gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  double new_message_time = msg->header.stamp.toSec();
  addValue(imu_angular_value_series, sum_imu_rotation, msg->angular_velocity.z, new_message_time);

  //double sum_imu_rotation = sum(imu_angular_value_series);
  //double sum_wheel_rotation = sum(wheel_angular_value_series);

  //double sum_pose_motion = sum(pose_linear_value_series);
  //double sum_wheel_motion = sum(wheel_linear_value_series);

  //ROS_WARN_STREAM_THROTTLE(1,"[antislip] ROT imu="<<sum_imu_rotation<<" wheel="<<sum_wheel_rotation<<
  //                " LIN pose="<<sum_pose_motion<<" wheel="<<sum_wheel_motion<<
  //                " sizes("<<imu_angular_value_series.size()<<","<<wheel_angular_value_series.size()<<","<<pose_linear_value_series.size()<<","<<wheel_linear_value_series.size()<<")");
  if(abs(sum_wheel_rotation) > relativeAngularDifference*abs(sum_imu_rotation) && abs(sum_wheel_rotation-sum_imu_rotation)>absoluteAngularDifference) {
    if(!angularSlip) {
      if(abs(sum_wheel_rotation-sum_imu_rotation)>absoluteAngularDifference*10) {
        ROS_ERROR_STREAM_THROTTLE(0.5,"[antislip] Angular slip unplausible difference. Integral rotations IMU="<<sum_imu_rotation<<" Wheel="<<sum_wheel_rotation);
      }else{
        ROS_WARN_STREAM("[antislip] Angular slip started. Integral rotations IMU="<<sum_imu_rotation<<" Wheel="<<sum_wheel_rotation);
        angularSlip = true;
        angularSlipStarted = msg->header.stamp;
      }
      //refresh accumulators
      sum_imu_rotation = sum(imu_angular_value_series);
      sum_wheel_rotation = sum(wheel_angular_value_series);
    }
  }else{
    if(angularSlip) {
      ROS_WARN_STREAM("[antislip] Angular slip ended in "<<(msg->header.stamp - angularSlipStarted).toSec()<<"s");
    }
    angularSlip = false;
  }

  if (angularSlip && (msg->header.stamp - angularSlipStarted).toSec() > angularAntislipTimeout) {
    ROS_WARN_STREAM_THROTTLE(0.5,"[antislip] Angular antislip activated. Integral rotations IMU="<<sum_imu_rotation<<" Wheel="<<sum_wheel_rotation);
    geometry_msgs::Twist cmd;
    cmd.linear.x = antislipVel;
    cmd.angular.z = 0.0;
    cmd_vel_pub.publish(cmd);
  }

}

// void onOdomIn(const nav_msgs::Odometry::ConstPtr &msg) {
//   double new_message_time = msg->header.stamp.toSec();
//   addValue(pose_linear_value_series,sum_pose_motion, msg->twist.twist.linear.x,new_message_time);
// }


void onTwistIn(const geometry_msgs::TwistStamped::ConstPtr &msg) {
  double new_message_time = msg->header.stamp.toSec();
  addValue(wheel_angular_value_series, sum_wheel_rotation, msg->twist.angular.z, new_message_time);
  //addValue(wheel_linear_value_series, sum_wheel_motion, msg->twist.linear.x, new_message_time);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "slip_detector");

  ros::NodeHandle n;
  ros::NodeHandle paramNh("~");

  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/antislip_vel", 1);

  ros::Subscriber imu_sub = paramNh.subscribe("imu_in", 10, onImu);
  ros::Subscriber twist_sub = paramNh.subscribe("twist_in", 10, onTwistIn);
  //ros::Subscriber odom_sub = paramNh.subscribe("odom3d_in", 10, onOdomIn);
  if (paramNh.param("integration_time", integrationTime, 2.0)) {
    ROS_INFO_STREAM("[antislip] Configured integration time: " << integrationTime);
  }
  if (paramNh.param("relative_angular_difference", relativeAngularDifference, 10.0)) {
    ROS_INFO_STREAM("[antislip] Configured relative angular difference: " << relativeAngularDifference);
  }
  if (paramNh.param("absolute_angular_difference", absoluteAngularDifference, 0.45)) {
    ROS_INFO_STREAM("[antislip] Configured absolute angular difference: " << absoluteAngularDifference);
  }
  if (paramNh.param("antislip_vel", antislipVel, 0.3)) {
    ROS_INFO_STREAM("[antislip] Configured antislip_vel: " << antislipVel);
  }
  if (paramNh.param("angular_antislip_timeout", angularAntislipTimeout, 1.0)) {
    ROS_INFO_STREAM("[antislip] Configured angular antislip temeout: " << angularAntislipTimeout);
  }

  //here we may need
  //1. cmd veleocity
  //2. actual cmd reported by each motor driver?
  //3. actual wheel velocities?

  //ros::Subscriber rear_state_sub =
  //    n.subscribe("/rear/hoverboard_driver/state", 0, onRearStateReceived, ros::TransportHints().tcpNoDelay(true));
  //ros::Subscriber front_state_sub =
  //    n.subscribe("/front/hoverboard_driver/state", 0, onFrontStateReceived, ros::TransportHints().tcpNoDelay(true));

  //#ifdef HOVERBOARD_ODOM
  //  ros::Subscriber rear_odom_sub =
  //    n.subscribe("/rear/hoverboard_velocity_controller/odom", 0, onRearOdomReceived, ros::TransportHints().tcpNoDelay(true));
  //  ros::Subscriber front_odom_sub =
  //    n.subscribe("/front/hoverboard_velocity_controller/odom", 0, onFrontOdomReceived, ros::TransportHints().tcpNoDelay(true));
  //#endif

  ros::spin();

  return 0;
}