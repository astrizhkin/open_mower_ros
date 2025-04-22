// Created by Clemens Elflein on 2/18/22, 5:37 PM.
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
#include "grid_map_cv/GridMapCvConverter.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "robot_localization/navsat_conversions.h"

// Slic3r Polygon tools
#include "ClipperUtils.hpp"
#include "grid_map_ros/PolygonRosConverter.hpp"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

// Rosbag for reading/writing the map to a file
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Include Messages
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PoseStamped.h"
#include "mower_map/MapArea.h"

// Include Service Messages
#include "mower_map/AddMowingAreaSrv.h"
#include "mower_map/AppendMapSrv.h"
#include "mower_map/ClearMapSrv.h"
#include "mower_map/ClearNavPointSrv.h"
#include "mower_map/ConvertToNavigationAreaSrv.h"
#include "mower_map/DeleteMowingAreaSrv.h"
#include "mower_map/GetDockingPointSrv.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/SetDockingPointSrv.h"
#include "mower_map/SetNavPointSrv.h"

#include "xbot_driver_gps/SetDatumSrv.h"

// Monitoring
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//JSON
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "xbot_msgs/Map.h"

ros::ServiceClient setDatumClient;

// Publishes the map as occupancy grid
ros::Publisher map_pub;

// Publishes the map as markers for rviz
ros::Publisher map_server_viz_array_pub;

// Publishes map for monitoring
ros::Publisher xbot_monitoring_map_pub;

// We store areas (i.e. navigation where robot is allowed to move and mowing where grass needs to be cut)
std::vector<mower_map::MapArea> areas;

// The recorded docking pose. Note that this is the pose from which the docking attempt is started
// I.e. the robot will drive to this pose and then drive forward
geometry_msgs::Pose docking_point;
bool has_docking_point = false;
bool show_fake_obstacle = false;
geometry_msgs::Pose fake_obstacle_pose;

// The grid map. This is built from the polygons loaded from the file.
grid_map::GridMap map;
std::string map_name;

geometry_msgs::Point base_point;
double base_point_e_;
double base_point_n_;
double base_point_u_;
std::string base_point_zone_;
bool has_base_point = false;

//area perimeter tolerance for simplification algorithm
double areaPerimeterTolerance = 0.05;
double mapResolution = 0.1;

/**
 * Convert a geometry_msgs::Polygon to a grid_map::Polygon.
 * This is needed in order to add recorded polys to the map.
 *
 * @param poly input poly
 * @param out result
 */
void fromMessage(geometry_msgs::Polygon &poly, grid_map::Polygon &out) {
  out.removeVertices();
  for (auto &point : poly.points) {
    grid_map::Position pos;
    pos.x() = point.x;
    pos.y() = point.y;
    out.addVertex(pos);
  }
}

/**
 * Publish map to xbot_monitoring
 */
void publishMapMonitoring() {
  ros::Time t1 = ros::Time::now();

  xbot_msgs::Map xb_map;
  xb_map.name = map_name;
  xb_map.mapWidth = map.getSize().x() * map.getResolution();
  xb_map.mapHeight = map.getSize().y() * map.getResolution();
  auto mapPos = map.getPosition();
  xb_map.mapCenterX = mapPos.x();
  xb_map.mapCenterY = mapPos.y();

  if(has_base_point) {
    xb_map.baseX = base_point.x;
    xb_map.baseY = base_point.y;
    xb_map.baseZ = base_point.z;
  }

  xb_map.dockX = docking_point.position.x;
  xb_map.dockY = docking_point.position.y;
  tf2::Quaternion q;
  tf2::fromMsg(docking_point.orientation, q);

  tf2::Matrix3x3 m(q);
  double unused1, unused2, yaw;

  m.getRPY(unused1, unused2, yaw);
  xb_map.dockHeading = yaw;

  for (const auto &area : areas) {
    xbot_msgs::MapArea xb_area;
    xb_area.name = area.name;
    xb_area.area = area.area;
    xb_area.area_type = area.area_type;
    xb_map.areas.push_back(xb_area);
  }

  xbot_monitoring_map_pub.publish(xb_map);
  ros::Time t2 = ros::Time::now();
  ROS_INFO_STREAM("[mower_map_service] Publish map in " << (t2 - t1).toSec() << " s");
}

/**
 * Publish map visualizations for rviz.
 */
void visualizeAreas() {
  ros::Time t1 = ros::Time::now();
  auto mapPos = map.getPosition();

  visualization_msgs::MarkerArray markerArray;

  grid_map::Polygon p;

  for (auto area : areas) {
    // Push it to mapAreas
    {
      // Create a marker
      fromMessage(area.area, p);
      std_msgs::ColorRGBA color;
      if (area.area_type == mower_map::MapArea::AREA_MOWING) {
        color.g = 1.0;
        color.a = 1.0;
      } else if (area.area_type == mower_map::MapArea::AREA_PROHIBITED) {
        color.r = 1.0;
        color.a = 1.0;
      } else if (area.area_type == mower_map::MapArea::AREA_NAVIGATION) {
      } else {
      }
      visualization_msgs::Marker marker;
      grid_map::PolygonRosConverter::toLineMarker(p, color, 0.05, 0, marker);

      marker.header.frame_id = "map";
      marker.ns = "mower_map_service";
      marker.id = markerArray.markers.size();
      marker.frame_locked = true;
      marker.pose.orientation.w = 1.0;

      markerArray.markers.push_back(marker);
    }
  }

  // Visualize Docking Point
  {
    std_msgs::ColorRGBA color;
    color.b = 1.0;
    color.a = 1.0;
    visualization_msgs::Marker marker;

    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.2;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color = color;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.pose = docking_point;
    ROS_INFO_STREAM("[mower_map_service] docking pose: " << docking_point);
    marker.header.frame_id = "map";
    marker.ns = "mower_map_service";
    marker.id = markerArray.markers.size() + 1;
    marker.frame_locked = true;
    markerArray.markers.push_back(marker);
  }

  map_server_viz_array_pub.publish(markerArray);
  ros::Time t2 = ros::Time::now();
  ROS_INFO_STREAM("[mower_map_service] Visualize map areas in " << (t2 - t1).toSec() << " s");
}

bool simplifyArea(double areaPerimeterTolerance, mower_map::MapArea &inArea) {
  // create slic3r ploygon
  Polygon poly;
  for (auto &pt : inArea.area.points) {
    poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
  }

  int pointsCountBefore = poly.points.size();
  // simplify
  Polygons simplifiedPolygons;
  poly.simplify(scale_(areaPerimeterTolerance), simplifiedPolygons);

  // select single max area polygon
  Polygon *singleSimplePolygon = nullptr;
  if (simplifiedPolygons.size() != 1) {
    ROS_WARN_STREAM("[mower_map_service] Self intersecting area found " << inArea.name);
    double maxArea = 0;
    for (auto &p : simplifiedPolygons) {
      double area = p.area();
      if (area > maxArea) {
        maxArea = area;
        singleSimplePolygon = &p;
      }
    }
    if (singleSimplePolygon == nullptr) {
      ROS_WARN_STREAM(
          "[mower_map_service] No simple polygon with positive area was selected after simplification. Original "
          << inArea.name << " area was " << unscale(unscale(poly.area())) << " m2");
      return false;
    } else {
      ROS_INFO_STREAM("[mower_map_service] Selected polygon with max positive area after simplification. Original "
                      << inArea.name << " area was " << unscale(unscale(poly.area())) << " m2 selected "
                      << unscale(unscale(singleSimplePolygon->area())) << " m2");
    }
  } else {
    singleSimplePolygon = &simplifiedPolygons.at(0);
  }
  ROS_INFO_STREAM("[mower_map_service] Reduced resolution of area " << inArea.name << " from " << pointsCountBefore
                                                                    << " to " << singleSimplePolygon->points.size());
  // convert back to geometry_msg
  inArea.area.points.clear();
  for (auto &pt : singleSimplePolygon->points) {
    geometry_msgs::Point32 gpt;
    gpt.x = unscale(pt.x);
    gpt.y = unscale(pt.y);
    gpt.z = 0;
    inArea.area.points.push_back(gpt);
  }
  return true;
}

/**
 * Uses the polygons stored in areas to build the final occupancy grid.
 *
 * First, the map is marked as completely occupied. Then navigation areas and mowing areas are marked as free.
 *
 * All obstacles are marked as occupied.
 *
 * Finally, a blur is applied to the map so that it is expensive, but not completely forbidden to drive near boundaries.
 */
void buildMap() {
  ros::Time t1 = ros::Time::now();
  // First, calculate the size of the map by finding the min and max values for x and y.
  float minX = FLT_MAX;
  float maxX = FLT_MIN;
  float minY = FLT_MAX;
  float maxY = FLT_MIN;

  // loop through all areas and calculate a size where everything fits
  for (const auto &area : areas) {
    for (auto pt : area.area.points) {
      minX = std::min(minX, pt.x);
      maxX = std::max(maxX, pt.x);
      minY = std::min(minY, pt.y);
      maxY = std::max(maxY, pt.y);
    }
  }

  // Enlarge the map by 1m in all directions.
  // This guarantees that even after blurring, the map has an occupied border.
  maxX += 1.0;
  minX -= 1.0;
  maxY += 1.0;
  minY -= 1.0;

  // Check, if the map was empty. If so, we'd create a huge map. Therefore we build an empty 10x10m map instead.
  if (areas.empty()) {
    maxX = 5.0;
    minX = -5.0;
    maxY = 5.0;
    minY = -5.0;
  }

  map = grid_map::GridMap({"navigation_area"});
  map.setFrameId("map");
  grid_map::Position origin;
  origin.x() = (maxX + minX) / 2.0;
  origin.y() = (maxY + minY) / 2.0;

  ROS_INFO_STREAM("[mower_map_service] Map Position: x=" << origin.x() << ", y=" << origin.y());
  ROS_INFO_STREAM("[mower_map_service] Map Size: x=" << (maxX - minX) << ", y=" << (maxY - minY));

  map.setGeometry(grid_map::Length(maxX - minX, maxY - minY), mapResolution, origin);
  map.setTimestamp(ros::Time::now().toNSec());

  map.clearAll();
  // mark all map as occupied (1.0)
  map["navigation_area"].setConstant(1.0);

  grid_map::Matrix &data = map["navigation_area"];
  // mark navigation and mowing as free (0.0)
  for (auto area : areas) {
    grid_map::Polygon poly;
    fromMessage(area.area, poly);
    if (area.area_type == mower_map::MapArea::AREA_MOWING || area.area_type == mower_map::MapArea::AREA_NAVIGATION) {
      for (grid_map::PolygonIterator iterator(map, poly); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);
        data(index[0], index[1]) = 0.0;
      }
    }
  }
  // then mark prohibited as occupied (1.0)
  for (auto area : areas) {
    grid_map::Polygon poly;
    fromMessage(area.area, poly);
    if (area.area_type == mower_map::MapArea::AREA_PROHIBITED) {
      for (grid_map::PolygonIterator iterator(map, poly); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);
        data(index[0], index[1]) = 1.0;
      }
    }
  }

  if (show_fake_obstacle) {
    grid_map::Polygon poly;
    tf2::Quaternion q;
    tf2::fromMsg(fake_obstacle_pose.orientation, q);

    tf2::Matrix3x3 m(q);
    double unused1, unused2, yaw;

    m.getRPY(unused1, unused2, yaw);

    Eigen::Vector2d front(cos(yaw), sin(yaw));
    Eigen::Vector2d left(-sin(yaw), cos(yaw));
    Eigen::Vector2d obstacle_pos(fake_obstacle_pose.position.x, fake_obstacle_pose.position.y);

    {
      grid_map::Position pos = obstacle_pos + 0.1 * left + 0.25 * front;
      poly.addVertex(pos);
    }
    {
      grid_map::Position pos = obstacle_pos + 0.2 * left - 0.1 * front;
      poly.addVertex(pos);
    }
    {
      grid_map::Position pos = obstacle_pos + 0.6 * left - 0.1 * front;
      poly.addVertex(pos);
    }
    {
      grid_map::Position pos = obstacle_pos + 0.6 * left + 0.7 * front;
      poly.addVertex(pos);
    }

    {
      grid_map::Position pos = obstacle_pos - 0.6 * left + 0.7 * front;
      poly.addVertex(pos);
    }
    {
      grid_map::Position pos = obstacle_pos - 0.6 * left - 0.1 * front;
      poly.addVertex(pos);
    }
    {
      grid_map::Position pos = obstacle_pos - 0.2 * left - 0.1 * front;
      poly.addVertex(pos);
    }
    {
      grid_map::Position pos = obstacle_pos - 0.1 * left + 0.25 * front;
      poly.addVertex(pos);
    }
    for (grid_map::PolygonIterator iterator(map, poly); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      data(index[0], index[1]) = 1.0;
    }
  }

  cv::Mat cv_map;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "navigation_area", CV_8UC1, cv_map);

  cv::blur(cv_map, cv_map, cv::Size(5, 5));

  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(cv_map, "navigation_area", map);

  nav_msgs::OccupancyGrid msg;
  grid_map::GridMapRosConverter::toOccupancyGrid(map, "navigation_area", 0.0, 1.0, msg);
  map_pub.publish(msg);
  ros::Time t2 = ros::Time::now();
  ROS_INFO_STREAM("[mower_map_service] Build map in " << (t2 - t1).toSec() << " s");

  publishMapMonitoring();
  visualizeAreas();
}

inline bool ends_with(std::string const & value, std::string const & ending)
{
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

/**
 * Saves the current polygons to a bag file.
 * We don't need to save the map, since we can easily build it again after loading.
 */

void saveMapToBagFile(const std::string &filename) {
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Write);

  if (has_docking_point) {
    bag.write("docking_point", ros::Time::now(), docking_point);
  }
  if(has_base_point) {
    bag.write("base_point", ros::Time::now(), base_point);
  }

  json areas_json;
  for (auto &area : areas) {
    bag.write("areas", ros::Time::now(), area);
  }

  bag.close();
}

void saveMapToGeoJsonFile(const std::string &filename) {
  json map_geojson;
  map_geojson["type"] = "FeatureCollection";
  
  json features_properties_geojson;
  features_properties_geojson["name"] = map_name;
  map_geojson["properties"] = features_properties_geojson;

  json features_geojson;
  if (has_docking_point) {
    json docking_point_feature_json;
    docking_point_feature_json["type"] = "Feature";
    json docking_point_geometry_json;
    docking_point_geometry_json["type"] = "Point";

    double e, n, u;
    e = docking_point.position.x + base_point_e_;
    n = docking_point.position.y + base_point_n_;
    u = docking_point.position.z + base_point_u_;
    double lat, lon;
    RobotLocalization::NavsatConversions::UTMtoLL(n, e, base_point_zone_, lat, lon);

    json docking_point_geometry_coordinates_json;
    docking_point_geometry_coordinates_json.push_back(lon);
    docking_point_geometry_coordinates_json.push_back(lat);
    docking_point_geometry_coordinates_json.push_back(u);
    docking_point_geometry_json["coordinates"] = docking_point_geometry_coordinates_json; 

    docking_point_feature_json["geometry"] = docking_point_geometry_json;

    json docking_point_properties_json;
    docking_point_properties_json["name"] = "docking_point";

    json docking_point_orientation_json;
    docking_point_orientation_json.push_back(docking_point.orientation.w);
    docking_point_orientation_json.push_back(docking_point.orientation.x);
    docking_point_orientation_json.push_back(docking_point.orientation.y);
    docking_point_orientation_json.push_back(docking_point.orientation.z);
    docking_point_properties_json["orientation"] = docking_point_orientation_json;

    docking_point_feature_json["properties"] = docking_point_properties_json;

    features_geojson.push_back(docking_point_feature_json);
  }
  if(has_base_point) {
    json base_point_feature_json;
    base_point_feature_json["type"] = "Feature";

    json base_point_geometry_json;
    base_point_geometry_json["type"] = "Point";

    json base_point_geometry_coordinates_json;
    base_point_geometry_coordinates_json.push_back(base_point.x);//lon
    base_point_geometry_coordinates_json.push_back(base_point.y);//lat
    base_point_geometry_coordinates_json.push_back(base_point.z);//height
    base_point_geometry_json["coordinates"] = base_point_geometry_coordinates_json; 

    base_point_feature_json["geometry"] = base_point_geometry_json;

    json base_point_properties_json;
    base_point_properties_json["name"] = "base_point";
    base_point_feature_json["properties"] = base_point_properties_json;

    features_geojson.push_back(base_point_feature_json);
  }


  for (auto &area : areas) {

    json area_feature_json;
    area_feature_json["type"] = "Feature";
    json area_geometry_json;
    area_geometry_json["type"] = "Polygon";
    json area_properties_json;
    area_properties_json["name"] = area.name;
    area_properties_json["area_type"] = area.area_type;
    area_properties_json["simplified"] = true;
    switch(area.area_type) {
      case mower_map::MapArea::AREA_MOWING:
        area_properties_json["fill"] = "#00FF00";
        break;
      case mower_map::MapArea::AREA_NAVIGATION:
        area_properties_json["fill"] = "#A0A0A0";
        break;
      case mower_map::MapArea::AREA_PROHIBITED:
        area_properties_json["fill"] = "#202020";
        break;
    }
    area_feature_json["properties"] = area_properties_json;

    json area_geometry_coordinates_json;
    json area_geometry_coordinates_polygon_json;
    for(auto &pt : area.area.points) {
      double e, n, u;
      e = pt.x + base_point_e_;
      n = pt.y + base_point_n_;
      u = pt.z + base_point_u_;
      double lat, lon;
      RobotLocalization::NavsatConversions::UTMtoLL(n, e, base_point_zone_, lat, lon);
      json pt_json;
      pt_json.push_back(lon);
      pt_json.push_back(lat);
      //pt_json.push_back(pt.x);
      //pt_json.push_back(pt.y);
      area_geometry_coordinates_polygon_json.push_back(pt_json);
    }
    area_geometry_coordinates_json.push_back(area_geometry_coordinates_polygon_json);
    area_geometry_json["coordinates"] = area_geometry_coordinates_json;

    area_feature_json["geometry"] = area_geometry_json;

    features_geojson.push_back(area_feature_json);
  }
 
  map_geojson["features"] = features_geojson;

  std::ofstream out(filename);
  out << map_geojson.dump(1);
  out.close();
}

/* Accept base point in lat/lon format */
void setBasePoint(double lon, double lat, double height) {
  base_point.x = lon;
  base_point.y = lat;
  base_point.z = height;
  has_base_point = true;

  std::string zone;
  double e, n;
  RobotLocalization::NavsatConversions::LLtoUTM(lat, lon, n, e, zone);
  base_point_e_ = e;
  base_point_n_ = n;
  base_point_u_ = height;
  base_point_zone_ = zone;
}

void saveMapToFile(const std::string &filename) {
  ros::Time t1 = ros::Time::now();
  if(ends_with(filename, ".bag")){
    saveMapToBagFile(filename);
  }else if(ends_with(filename, ".geojson")){
    saveMapToGeoJsonFile(filename);
  }else{
    ROS_ERROR_STREAM("[mower_map_service] Save to unsupported file format " << filename << " (expected *.bag or *.geojson)");
  }
  ros::Time t2 = ros::Time::now();
  ROS_INFO_STREAM("[mower_map_service] Saved " << areas.size() << " areas to file in " << (t2 - t1).toSec() << " s");
}

void saveMap() {
  saveMapToFile("map.bag");
  saveMapToFile("map.geojson");
}

/**
 * Load the polygons from the bag file and build a map.
 *
 * @param filename The file to load.
 * @param append True to append the loaded map to the current one.
 */

void readMapFromBagFile(const std::string &filename) {
  rosbag::Bag bag;
  try {
    bag.open(filename);
  } catch (rosbag::BagIOException &e) {
    ROS_WARN("[mower_map_service] Error opening stored mowing areas bag.");
    return;
  }
  map_name = filename;
  {
    rosbag::View view(bag, rosbag::TopicQuery("areas"));
    for (rosbag::MessageInstance const m : view) {
      mower_map::MapAreaPtr areaPtr = m.instantiate<mower_map::MapArea>();
      if (areaPerimeterTolerance == 0 || simplifyArea(areaPerimeterTolerance, *areaPtr)) {
        areas.push_back(*areaPtr);
      }
    }
  }
  {
    has_docking_point = false;
    rosbag::View view(bag, rosbag::TopicQuery("docking_point"));
    for (rosbag::MessageInstance const m : view) {
      auto pt = m.instantiate<geometry_msgs::Pose>();
      docking_point = *pt;
      has_docking_point = true;
      break;
    }
    if (!has_docking_point) {
      geometry_msgs::Pose empty;
      empty.orientation.w = 1.0;
      docking_point = empty;
    }
  }
  {
    has_base_point = false;
    rosbag::View view(bag, rosbag::TopicQuery("base_point"));
    for (rosbag::MessageInstance const m : view) {
      auto pt = m.instantiate<geometry_msgs::Point>();
      setBasePoint(pt->x, pt->y, pt->z);
      break;
    }
  }
}

std::string readMapFromGeoJsonFile(const std::string &filename) {
  std::ifstream f(filename);
  json map_json;
  try {
    map_json = json::parse(f);
  } catch (json::parse_error& ex) {
    ROS_WARN_STREAM("[mower_map_service] Error opening stored mowing areas json: " << ex.what() << " at byte " << (int)ex.byte);
    return "GeoJSON parse error";
  }

  if(map_json["type"]!="FeatureCollection") {
    return "root element is not a FeatureCollection";
  }
  if(!map_json.contains("features")) {
    return "root element must contain 'features'";
  }
  if(map_json.contains("properties")){
    json map_properties_json = map_json["properties"];
    if(map_properties_json.contains("name")) {
      map_name = map_properties_json["name"];
    }
  }
  json features_json = map_json["features"];
  //lookup the base_point
  has_base_point = false;
  for(auto &feature_json : features_json) {
    if(feature_json["type"] != "Feature") {
      ROS_WARN_STREAM("[mower_map_service] Load GeoJSON skipping " << feature_json["type"] << " element");
      continue;
    }
    if(!feature_json.contains("geometry")){
      ROS_WARN_STREAM("[mower_map_service] Load GeoJSON skipping Feature element without 'geometry'");
      continue;
    }
    json geometry_json = feature_json["geometry"];
    if(!geometry_json.contains("coordinates")){
      ROS_WARN_STREAM("[mower_map_service] Load GeoJSON skipping Feature element without 'geometry' 'coordinates'");
      continue;
    }
    json coordinates_json = geometry_json["coordinates"];
    if(!geometry_json.contains("type")) {
      ROS_WARN_STREAM("[mower_map_service] Load GeoJSON skipping Feature element without 'geometry' 'type'");
      continue;
    }
    if(!feature_json.contains("properties")) {
      ROS_WARN_STREAM("[mower_map_service] Load GeoJSON skipping Feature element without 'properties'");
      continue;
    }
    json properties_json = feature_json["properties"];

    if(properties_json["name"] == "base_point"){
      //we found it
      if(geometry_json["type"] != "Point"){
        return "base_point 'geometry' element must be of 'Point' type";
      }
      if(!coordinates_json.is_array() || coordinates_json.size()!=3){
        return "base_point 'coordinates' element must be an array of size 3";
      }
      double lon = coordinates_json[0];
      double lat = coordinates_json[1];
      double height = coordinates_json[2];
      ROS_INFO_STREAM("[mower_map_service] Load GeoJSON found a base point " << lon << "E, " << lat << "N, " << height);
      setBasePoint(lon, lat, height);
      break;
    }
  }
  if(!has_base_point) {
    return "No base_point found (features[ type = Feature , properties[name] = base_point ] )";
  }

  //lookup for docking point and polygons
  has_docking_point = false;
  for(auto &feature_json : features_json) {
    if(feature_json["type"] != "Feature") {
      ROS_WARN_STREAM("[mower_map_service] Load GeoJSON skipping " << feature_json["type"] << " element");
      continue;
    }
    if(!feature_json.contains("geometry")){
      ROS_WARN_STREAM("[mower_map_service] Load GeoJSON skipping Feature element without 'geometry'");
      continue;
    }
    json geometry_json = feature_json["geometry"];
    if(!geometry_json.contains("coordinates")){
      ROS_WARN_STREAM("[mower_map_service] Load GeoJSON skipping Feature element without 'geometry' 'coordinates'");
      continue;
    }
    json coordinates_json = geometry_json["coordinates"];
    if(!geometry_json.contains("type")) {
      ROS_WARN_STREAM("[mower_map_service] Load GeoJSON skipping Feature element without 'geometry' 'type'");
      continue;
    }
    if(!feature_json.contains("properties")) {
      ROS_WARN_STREAM("[mower_map_service] Load GeoJSON skipping Feature element without 'properties'");
      continue;
    }
    json properties_json = feature_json["properties"];

    if(properties_json["name"] == "base_point"){
      continue;
    } else if(properties_json["name"] == "docking_point"){
      //we found docking point
      if(geometry_json["type"] != "Point"){
        return "docking_point 'geometry' element must be of 'Point' type";
      }
      if(!coordinates_json.is_array() || coordinates_json.size()!=3){
        return "docking_point 'coordinates' element must be an array of size 3";
      }
      double lon = coordinates_json[0];
      double lat = coordinates_json[1];
      double height = coordinates_json[2];
      if(!properties_json.contains("orientation")){
        return "docking_point element must have orientation property";
      }
      json orientation_json = properties_json["orientation"];
      if(!orientation_json.is_array() || orientation_json.size()!=4){
        return "docking_point orientation property must be an array of size 4";
      }

      double e, n, u;
      std::string zone;
      RobotLocalization::NavsatConversions::LLtoUTM(lat, lon, n, e, zone);
      e = e - base_point_e_;
      n = n - base_point_n_;
      u = height - base_point_u_;

      docking_point.position.x = e;
      docking_point.position.y = n;
      docking_point.position.z = u;

      docking_point.orientation.w = orientation_json[0];
      docking_point.orientation.x = orientation_json[1];
      docking_point.orientation.y = orientation_json[2];
      docking_point.orientation.z = orientation_json[3];
  
      ROS_INFO_STREAM("[mower_map_service] Load GeoJSON found a docking point " << lon << "E, " << lat << "N, " << height);
      has_docking_point = true;
    } else if(properties_json.contains("area_type")) {
      //we found area
      if(!properties_json["area_type"].is_number_integer()){
        ROS_WARN_STREAM("[mower_map_service] Load GeoJSON skipping area with non-integer area_type property");
        continue;
      }
      if(geometry_json["type"] != "Polygon"){
        ROS_WARN_STREAM("[mower_map_service] Load GeoJSON skipping area on unknown geometry type " << geometry_json["type"]);
        continue;
      }
      if(!properties_json.contains("name") || !properties_json["name"].is_string()) {
        ROS_WARN_STREAM("[mower_map_service] Load GeoJSON skipping area without name");
        continue;
      }
      if(!coordinates_json.is_array() || coordinates_json.size()!=1){
        ROS_WARN_STREAM("[mower_map_service] Load GeoJSON skipping area Polygon with non-array 'coordinates' or array size > 1");
        continue;
      }
      mower_map::MapArea mapArea;
      mapArea.name = properties_json["name"];
      mapArea.area_type = properties_json["area_type"];
      bool simplified = false;
      if(properties_json.contains("simplified") && properties_json["simplified"].is_boolean()){
        simplified = properties_json["simplified"];
      }
      for(auto &coordinates_array_json : coordinates_json) {
        if(!coordinates_array_json.is_array()) {
          ROS_WARN_STREAM("[mower_map_service] Load GeoJSON skipping area Polygon with non-array of array 'coordinates'");
          continue;
        }
        for(auto &coordinates_array_array_json : coordinates_array_json) {
          if(!coordinates_array_array_json.is_array() || coordinates_array_array_json.size()!=2) {
            ROS_WARN_STREAM("[mower_map_service] Load GeoJSON skipping area Polygon with non-array of array of array of size 2 'coordinates'");
            continue;
          }
          double lon = coordinates_array_array_json[0];
          double lat = coordinates_array_array_json[1];

          double e, n, u;
          std::string zone;
          RobotLocalization::NavsatConversions::LLtoUTM(lat, lon, n, e, zone);
          e = e - base_point_e_;
          n = n - base_point_n_;
          //u = height - base_point_u_;
    
          geometry_msgs::Point32 pt;
          pt.x = e;
          pt.y = n;
          mapArea.area.points.push_back(pt);
        }
      }

      if (areaPerimeterTolerance == 0 || simplified || simplifyArea(areaPerimeterTolerance, mapArea)) {
        areas.push_back(mapArea);
      }
    } else {
      ROS_WARN_STREAM("[mower_map_service] Load GeoJSON skipping Feature element without 'area_type' property or 'name' is unknown");
      continue;
    }
  }
  return "ok";
}

void readMapFromFile(const std::string &filename, bool append = false) {
  if (!append) {
    areas.clear();
  }
  ros::Time t1 = ros::Time::now();
  ROS_INFO_STREAM("[mower_map_service] Start load map from file " << filename);
  if(ends_with(filename,".bag")) {
    readMapFromBagFile(filename);
    //init 
    if(!has_base_point) {
      setBasePoint(29.43348637, 60.97728809, 56);
    }
    saveMapToFile("autoload_bag.geojson");
    saveMapToFile("autoload_bag.bag");
  } else if(ends_with(filename,".geojson")) {
    std::string errorMsg = readMapFromGeoJsonFile(filename);
    if(errorMsg!="ok") {
      ROS_ERROR_STREAM("[mower_map_service] Load GeoJSON failed with message " << errorMsg);
    }
    saveMapToFile("autoload_geojson.geojson");
    saveMapToFile("autoload_geojson.bag");
  } else {
    ROS_ERROR_STREAM("[mower_map_service] Unsupported map file " << filename << " (expected *.bag or *.geojson)");
  }
  if(!has_docking_point) {
    ROS_WARN_STREAM("[mower_map_service] Loaded map missing docking_point");
  }
  if(setDatumClient.exists()) {
    xbot_driver_gps::SetDatumSrv srv;
    if(has_base_point) {
      srv.request.revert_default = false;
      srv.request.longitute = base_point.x;
      srv.request.latitude = base_point.y;
      srv.request.height = base_point.z;
      ROS_INFO_STREAM("[mower_map_service] Publishing base_point");
      
      if(!setDatumClient.call(srv)) {
        ROS_WARN_STREAM("[mower_map_service] base_point was not published!");
      }
    } else {
      srv.request.revert_default = true;
      ROS_INFO_STREAM("[mower_map_service] Reverting base_point to defaults");
      if(!setDatumClient.call(srv)) {
        ROS_WARN_STREAM("[mower_map_service] base_point revert fail");
      }
    }
  } else {
    ROS_WARN_STREAM("[mower_map_service] Skip publishing base_point because setDatum service is not available");
  }
  ros::Time t2 = ros::Time::now();
  ROS_INFO_STREAM("[mower_map_service] Loaded " << areas.size() << " areas from file in " << (t2 - t1).toSec() << " s");
}


void generateTestMap() {
  areas.clear();

  double nav_hRad = 12.0;
  double nav_vRad = 11.0;

  double mowing_hRad = 7.0;
  double mowing_vRad = 6.0;
  double prohibited_hRad = 2.0;
  double prohibited_vRad = 2.0;
  double prohibited1_h_offset = 10;  //<=4 inside, 5 touch outline inside, 6-8 cross, 9 touch outline outside, >=10 outside
  double prohibited1_v_offset = 0;

  double prohibited2_h_offset = 6;  //<=4 inside, 5 touch outline inside, 6-8 cross, 9 touch outline outside, >=10 outside
  double prohibited2_v_offset = 0;

  int point_count = 20;

  {
    mower_map::MapArea nav_area;
    nav_area.area_type = mower_map::MapArea::AREA_NAVIGATION;
    nav_area.name = "nav1";

    mower_map::MapArea mowing_area;
    mowing_area.area_type = mower_map::MapArea::AREA_MOWING;
    mowing_area.name = "mowing1";

    mower_map::MapArea prohibited1_area;
    prohibited1_area.area_type = mower_map::MapArea::AREA_PROHIBITED;
    prohibited1_area.name = "prohibited1";

    mower_map::MapArea prohibited2_area;
    prohibited2_area.area_type = mower_map::MapArea::AREA_PROHIBITED;
    prohibited2_area.name = "prohibited2";

    for (int i = 0; i < point_count; i++) {
      double angle = 2 * M_PI * i / point_count;
      while (angle > M_PI) {
        angle -= 2 * M_PI;
      }
      geometry_msgs::Point32 navPt;
      geometry_msgs::Point32 mowingPt;
      geometry_msgs::Point32 prohibited1Pt;
      geometry_msgs::Point32 prohibited2Pt;

      navPt.x = cos(angle) * nav_hRad;
      navPt.y = sin(angle) * nav_vRad;

      mowingPt.x = cos(angle) * mowing_hRad;
      mowingPt.y = sin(angle) * mowing_vRad;

      prohibited1Pt.x = cos(angle) * prohibited_hRad + prohibited1_h_offset;
      prohibited1Pt.y = sin(angle) * prohibited_vRad + prohibited1_v_offset;

      prohibited2Pt.x = cos(angle) * prohibited_hRad + prohibited2_h_offset;
      prohibited2Pt.y = sin(angle) * prohibited_vRad + prohibited2_v_offset;

      // double tangentAngle = atan2(vRad * cos(angle), -hRad * sin(angle));
      // docking_pose_stamped_front.pose.orientation = tf2::toMsg(tf2::Quaternion(0,0,tangentAngle));

      nav_area.area.points.push_back(navPt);
      mowing_area.area.points.push_back(mowingPt);
      prohibited1_area.area.points.push_back(prohibited1Pt);
      prohibited2_area.area.points.push_back(prohibited2Pt);
    }

    geometry_msgs::Pose dockingPt;
    dockingPt.position.x = 0;
    dockingPt.position.y = 0;

    docking_point = dockingPt;
    areas.push_back(nav_area);
    areas.push_back(mowing_area);
    areas.push_back(prohibited1_area);
    areas.push_back(prohibited2_area);
  }

  ROS_INFO_STREAM("[mower_map_service] Generated test map");
}

bool addMowingArea(mower_map::AddMowingAreaSrvRequest &req, mower_map::AddMowingAreaSrvResponse &res) {
  ROS_INFO_STREAM("[mower_map_service] Got addMowingArea call");

  if (req.area.name.empty()) {
    int areaNameIndex = -1;
    bool nameIsBusy;
    std::string newNameStr;

    do {
      areaNameIndex++;
      nameIsBusy = false;
      std::ostringstream newName;
      if (req.area.area_type == mower_map::MapArea::AREA_NAVIGATION) {
        newName << "nav-" << areaNameIndex;
      } else if (req.area.area_type == mower_map::MapArea::AREA_MOWING) {
        newName << "mow-" << areaNameIndex;
      } else if (req.area.area_type == mower_map::MapArea::AREA_PROHIBITED) {
        newName << "pro-" << areaNameIndex;
      } else {
        newName << "unk-" << areaNameIndex;
      }
      newNameStr = newName.str();

      for (auto &area : areas) {
        if (area.name == newNameStr) {
          nameIsBusy = true;
          break;
        }
      }
    } while (nameIsBusy);

    req.area.name = newNameStr;
    ROS_INFO_STREAM("[mower_map_service] New area name " << req.area.name);
  }

  if (areaPerimeterTolerance > 0 && !simplifyArea(areaPerimeterTolerance, req.area)) {
    return false;
  }

  areas.push_back(req.area);

  saveMap();
  buildMap();
  return true;
}

bool getMowingArea(mower_map::GetMowingAreaSrvRequest &req, mower_map::GetMowingAreaSrvResponse &res) {
  ROS_INFO_STREAM("[mower_map_service] Got getMowingArea call with index: " << req.index);

  int mowingAreaIndex = 0;
  bool found = false;
  for (auto &area : areas) {
    if (area.area_type == mower_map::MapArea::AREA_MOWING) {
      if (mowingAreaIndex == req.index) {
        res.area = area;
        found = true;
      }
      mowingAreaIndex++;
    } else if (area.area_type == mower_map::MapArea::AREA_PROHIBITED) {
      res.prohibited_areas.push_back(area.area);
    }
  }

  if (!found) {
    ROS_ERROR_STREAM("[mower_map_service] No mowing area with index: " << req.index);
    return false;
  }

  return true;
}

bool deleteMowingArea(mower_map::DeleteMowingAreaSrvRequest &req, mower_map::DeleteMowingAreaSrvResponse &res) {
  ROS_INFO_STREAM("[mower_map_service] Got delete area call with index: " << req.index);

  if (req.index >= areas.size()) {
    ROS_ERROR_STREAM("[mower_map_service] No area with index: " << req.index);
    return false;
  }

  areas.erase(areas.begin() + req.index);

  saveMap();
  buildMap();

  return true;
}

bool convertToNavigationArea(mower_map::ConvertToNavigationAreaSrvRequest &req,
                             mower_map::ConvertToNavigationAreaSrvResponse &res) {
  ROS_INFO_STREAM("[mower_map_service] Got convert to nav area call with index: " << req.index);

  if (req.index >= areas.size()) {
    ROS_ERROR_STREAM("[mower_map_service] No mowing area with index: " << req.index);
    return false;
  }

  areas[req.index].area_type = mower_map::MapArea::AREA_NAVIGATION;

  saveMap();
  buildMap();

  return true;
}

bool appendMapFromFile(mower_map::AppendMapSrvRequest &req, mower_map::AppendMapSrvResponse &res) {
  ROS_INFO_STREAM("[mower_map_service] Appending maps from: " << req.bagfile);

  readMapFromFile(req.bagfile, true);

  saveMap();
  buildMap();

  return true;
}

bool setDockingPoint(mower_map::SetDockingPointSrvRequest &req, mower_map::SetDockingPointSrvResponse &res) {
  ROS_INFO_STREAM("[mower_map_service] Setting Docking Point");

  docking_point = req.docking_pose;
  has_docking_point = true;

  saveMap();
  buildMap();

  return true;
}

bool getDockingPoint(mower_map::GetDockingPointSrvRequest &req, mower_map::GetDockingPointSrvResponse &res) {
  ROS_INFO_STREAM("[mower_map_service] Getting Docking Point");

  res.docking_pose = docking_point;

  return has_docking_point;
}

bool setNavPoint(mower_map::SetNavPointSrvRequest &req, mower_map::SetNavPointSrvResponse &res) {
  ROS_INFO_STREAM("[mower_map_service] Setting Nav Point");

  fake_obstacle_pose = req.nav_pose;
  show_fake_obstacle = true;

  buildMap();

  return true;
}

bool clearNavPoint(mower_map::ClearNavPointSrvRequest &req, mower_map::ClearNavPointSrvResponse &res) {
  ROS_INFO_STREAM("[mower_map_service] Clearing Nav Point");

  if (show_fake_obstacle) {
    show_fake_obstacle = false;

    buildMap();
  }

  return true;
}

bool clearMap(mower_map::ClearMapSrvRequest &req, mower_map::ClearMapSrvResponse &res) {
  ROS_INFO_STREAM("[mower_map_service] Clearing Map");

  areas.clear();
  has_docking_point = false;

  saveMap();
  buildMap();
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mower_map_service");
  has_docking_point = false;
  ros::NodeHandle n;
  ros::NodeHandle paramNh("~");

  if (!paramNh.getParam("areaPerimeterTolerance", areaPerimeterTolerance)) {
    ROS_WARN_STREAM("[mower_map_service] No areaPerimeterTolerance parameter specified. Using default "
                    << areaPerimeterTolerance);
  }
  if (areaPerimeterTolerance <= 0) {
    areaPerimeterTolerance = 0;
    ROS_INFO_STREAM(
        "[mower_map_service] areaPerimeterTolerance is set to 0. Area perimiter simplification is disabled.");
  } else if (areaPerimeterTolerance > 1) {
    ROS_FATAL_STREAM("[mower_map_service] areaPerimeterTolerance parameter is out of range [0-1]m");
    return 1;
  }

  if (!paramNh.getParam("mapResolution", mapResolution)) {
    ROS_WARN_STREAM("[mower_map_service] No mapResolution parameter specified. Using default " << mapResolution);
  }
  if (mapResolution <= 0 || mapResolution > 1) {
    ROS_FATAL_STREAM("[mower_map_service] mapResolution parameter is out of range (0-1]m");
    return 1;
  }

  map_pub = n.advertise<nav_msgs::OccupancyGrid>("mower_map_service/map", 10, true);
  map_server_viz_array_pub = n.advertise<visualization_msgs::MarkerArray>("mower_map_service/map_viz", 10, true);
  xbot_monitoring_map_pub = n.advertise<xbot_msgs::Map>("xbot_monitoring/map", 10, true);


  setDatumClient = n.serviceClient<xbot_driver_gps::SetDatumSrv>("xbot_driver_gps/set_datum");
  ros::Rate r(1.0);

  ROS_INFO("[mower_map_service] Waiting for setDatum service");
  if (!setDatumClient.waitForExistence(ros::Duration(8.0, 0.0))) {
    ROS_WARN("[mower_map_service] setDatum service not found");
  }


  // Load the default map file
  readMapFromFile("map.geojson");
  //readMapFromFile("load_bag.geojson");
  // generateTestMap();

  buildMap();

  ros::ServiceServer add_area_srv = n.advertiseService("mower_map_service/add_mowing_area", addMowingArea);
  ros::ServiceServer get_area_srv = n.advertiseService("mower_map_service/get_mowing_area", getMowingArea);
  ros::ServiceServer delete_area_srv = n.advertiseService("mower_map_service/delete_mowing_area", deleteMowingArea);
  ros::ServiceServer append_maps_srv = n.advertiseService("mower_map_service/append_maps", appendMapFromFile);
  ros::ServiceServer convert_maps_srv = n.advertiseService("mower_map_service/convert_to_navigation_area", convertToNavigationArea);
  ros::ServiceServer set_docking_point_srv = n.advertiseService("mower_map_service/set_docking_point", setDockingPoint);
  ros::ServiceServer get_docking_point_srv = n.advertiseService("mower_map_service/get_docking_point", getDockingPoint);
  ros::ServiceServer set_nav_point_srv = n.advertiseService("mower_map_service/set_nav_point", setNavPoint);
  ros::ServiceServer clear_nav_point_srv = n.advertiseService("mower_map_service/clear_nav_point", clearNavPoint);
  ros::ServiceServer clear_map_srv = n.advertiseService("mower_map_service/clear_map", clearMap);

  ros::spin();
  return 0;
}
