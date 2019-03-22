#pragma once
#include <ros/ros.h>
#include "autoware_msgs/DynamicObjectWithFeatureArray.h"
#include "autoware_msgs/DynamicObjectArray.h"
#include "autoware_msgs/Shape.h"

class DynamicObjectVisualizer
{
private: // ros
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  void dynamicObjectWithFeatureCallback(const autoware_msgs::DynamicObjectWithFeatureArray::ConstPtr &input_msg);
  void dynamicObjectCallback(const autoware_msgs::DynamicObjectArray::ConstPtr &input_msg);
  bool calcBoundingBoxLineList(const autoware_msgs::Shape &shape, std::vector<geometry_msgs::Point> &points);
  bool calcCylinderLineList(const autoware_msgs::Shape &shape, std::vector<geometry_msgs::Point> &points);
  bool calcPolygonLineList(const autoware_msgs::Shape &shape, std::vector<geometry_msgs::Point> &points);
  bool getLabel(const autoware_msgs::Semantic &semantic, std::string &label);

  bool only_known_objects_;

public:
  DynamicObjectVisualizer();
  virtual ~DynamicObjectVisualizer(){}
};