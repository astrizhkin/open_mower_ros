//
// Created by Clemens Elflein on 22.02.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/GetMowingAreasSrv.h"
#include "ros/ros.h"
#include "slic3r_coverage_planner/PlanPath.h"

ros::ServiceClient pathClient, getAreaClient, getAreasClient;

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner_test");

  ros::NodeHandle n;
  ros::NodeHandle paramNh("~");

  int area_index = paramNh.param("area_index", 0);
  int outline_count = paramNh.param("outline_count", 4);

  ros::Publisher path_pub;

  path_pub = n.advertise<nav_msgs::Path>("mower_logic/mowing_path", 100, true);

  pathClient = n.serviceClient<slic3r_coverage_planner::PlanPath>("slic3r_coverage_planner/plan_path");
  getAreaClient = n.serviceClient<mower_map::GetMowingAreaSrv>("mower_map_service/get_mowing_area");
  getAreasClient = n.serviceClient<mower_map::GetMowingAreasSrv>("mower_map_service/get_mowing_areas");

  ROS_INFO("Waiting for map server");
  if (!getAreaClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Map area service not found.");
    return 2;
  }
  if (!getAreasClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Map areas service not found.");
    return 2;
  }

    int index = 0;
    //while(true) {
        mower_map::GetMowingAreasSrv getAreas;

        if (!getAreasClient.call(getAreas)) {
            ROS_ERROR_STREAM("Error loading mowing area with index " << index);
            return 1;
        }
 
        mower_map::GetMowingAreaSrv getArea;
        getArea.request.name = getAreas.response.areas[0].name;

        if (!getAreaClient.call(getArea)) {
            ROS_ERROR_STREAM("Error loading mowing area with index " << index);
            return 1;
        }

        slic3r_coverage_planner::PlanPath pathSrv;
        pathSrv.request.angle = 0;
        pathSrv.request.outline_count = 4;
        pathSrv.request.outline = getArea.response.area.area;
        pathSrv.request.holes = getArea.response.prohibited_areas;
        pathSrv.request.fill_type = slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR;
        pathSrv.request.outline_overlap_count = 0;
        pathSrv.request.distance = 0.3;

        if (!pathClient.call(pathSrv)) {
            ROS_ERROR_STREAM("Error getting path area with index " << index);
            return 1;
        }
        index++;
    //}
    return 0;
}
