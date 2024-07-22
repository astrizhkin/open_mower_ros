#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <ftc_local_planner/PID.h>
//#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>
#include "tf2_eigen/tf2_eigen.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>

//#include <mbf_costmap_core/costmap_controller.h>
//#include <visualization_msgs/Marker.h>

tf2_msgs::TFMessage last_transform;
nav_msgs::Odometry last_odom;
geometry_msgs::PoseStamped last_pose;
ros::Publisher error_pub;
tf2_ros::Buffer tfBuffer;

/*ros::Publisher pose_pub;
geometry_msgs::PoseWithCovarianceStamped out;
std::string frame;


void pose_received(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
    out.header = msg->header;
    out.pose = msg->pose;
    out.pose.pose.position.z = 0;
    out.header.frame_id = frame;
    pose_pub.publish(out);
}*/

void onTransformation(const tf2_msgs::TFMessage::ConstPtr &msg) {
    ROS_INFO_STREAM("FTCLocalPlannerTest: onTransformation");
    last_transform.transforms = msg -> transforms;
    //last_transform.transform = msg -> transform;
    //last_transform.child_frame_id = msg -> child_frame_id;
}

void onOdom(const nav_msgs::Odometry::ConstPtr &msg) {
    ROS_INFO_STREAM("FTCLocalPlannerTest: onOdom");
    last_odom.header = msg -> header;
    last_odom.pose = msg -> pose;
    last_odom.child_frame_id = msg -> child_frame_id;
    last_odom.twist = msg -> twist;
}

void onControlPoint(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    ROS_INFO_STREAM("FTCLocalPlannerTest: onControlPoint");
    last_pose.header = msg -> header;
    last_pose.pose = msg -> pose;

    Eigen::Affine3d current_control_point;
    tf2::fromMsg(msg->pose, current_control_point);

    Eigen::Affine3d local_control_point;
    //transform
    geometry_msgs::TransformStamped map_to_base = tfBuffer.lookupTransform("base_link", "map", ros::Time(), ros::Duration(1.0));
    //map_to_base.transform.rotation.
    //geometry_msgs::TransformStamped transform = last_transform.transforms.at(0);
    //tf2::fromMsg(last_transform);
    tf2::Transform t;
    tf2::fromMsg(map_to_base.transform,t);
    tf2::Matrix3x3 m = t.getBasis();
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    m.setRPY(0,0,yaw);
    t.setBasis(m);
    //tf2::Quaternion q = t.getRotation();
    //t = t.inverse();
    map_to_base.transform = tf2::toMsg(t);
    
    tf2::doTransform(current_control_point, local_control_point, map_to_base);

    ftc_local_planner::PID p;
    p.lat_err = local_control_point.translation().y();
    p.lon_err = local_control_point.translation().x();
    p.ang_err = local_control_point.rotation().eulerAngles(0, 1, 2).z();

    ROS_INFO_STREAM("FTCLocalPlannerTest: Angular err: " << p.ang_err );

    error_pub.publish(p);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ftc_planner_test");

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    ros::Subscriber global_point_sub = n.subscribe("/move_base_flex/FTCPlanner/global_point", 0, onControlPoint);
    ros::Subscriber tf_sub = n.subscribe("/tf", 0, onTransformation);
    ros::Subscriber odom_sub = n.subscribe("/xbot_positioning/odom_out", 0, onOdom);

    error_pub = paramNh.advertise<ftc_local_planner::PID>("test_error", 10, false);

    tf2_ros::TransformListener tfListener(tfBuffer);

/*    geometry_msgs::TransformStamped map_to_base;
    map_to_base.transform.rotation.x = -0.014;
    map_to_base.transform.rotation.y = -0.009;
    map_to_base.transform.rotation.z =  0.323;
    map_to_base.transform.rotation.w = -0.946;
    map_to_base.transform.translation.x = -12.472;
    map_to_base.transform.translation.y = 5.585;
    map_to_base.transform.translation.z = 0.04;

    geometry_msgs::Pose pose;
    pose.position.x = -12.38;
    pose.position.y = 5.651;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = -0.69;
    pose.orientation.w = 0.724;
    Eigen::Affine3d current_control_point;
    tf2::fromMsg(pose, current_control_point);

    Eigen::Affine3d local_control_point;

    tf2::doTransform(current_control_point, local_control_point, map_to_base);

    double lat_error = local_control_point.translation().y();
    double lon_error = local_control_point.translation().x();
    double angle_error = local_control_point.rotation().eulerAngles(0, 1, 2).z();

    tf2::Quaternion q;
    tf2::fromMsg(map_to_base.transform.rotation,q);
    //tf2::quatRotate(q,);
    tf2::Matrix3x3 m(q);
        
    double r,p,y;
    m.getRPY(r,p,y);
    ROS_INFO_STREAM("FTCLocalPlannerROS: Angular err: EA " << angle_error << " +pi" << (angle_error + M_PI) << " Y " << y );*/
        ros::spin();

    return 0;

}

