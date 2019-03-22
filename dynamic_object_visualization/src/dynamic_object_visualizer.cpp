#include "dynamic_object_visualization/dynamic_object_visualizer.h"
#include <string>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

DynamicObjectVisualizer::DynamicObjectVisualizer() : nh_(""), private_nh_("~")
{
  bool with_feature;
  private_nh_.param<bool>("with_feature", with_feature, true);
  private_nh_.param<bool>("only_known_objects", only_known_objects_, true);
  if (with_feature)
    sub_ = nh_.subscribe("input", 1, &DynamicObjectVisualizer::dynamicObjectWithFeatureCallback, this);
  else
    sub_ = nh_.subscribe("input", 1, &DynamicObjectVisualizer::dynamicObjectCallback, this);
  pub_ = nh_.advertise<visualization_msgs::MarkerArray>("output", 1, true);
}

void DynamicObjectVisualizer::dynamicObjectWithFeatureCallback(const autoware_msgs::DynamicObjectWithFeatureArray::ConstPtr &input_msg)
{
  visualization_msgs::MarkerArray output;
  int id = 0;
  // shape
  for (size_t i = 0; i < input_msg->feature_objects.size(); ++i)
  {
    if (only_known_objects_)
    {
      if (input_msg->feature_objects.at(i).object.semantic.type == autoware_msgs::Semantic::UNKNOWN)
        continue;
    }
    visualization_msgs::Marker marker;
    marker.header = input_msg->header;
    marker.id = id++;
    marker.ns = std::string("shape");
    if (input_msg->feature_objects.at(i).object.shape.type == autoware_msgs::Shape::BOUNDING_BOX)
    {
      marker.type = visualization_msgs::Marker::LINE_LIST;
      if (!calcBoundingBoxLineList(input_msg->feature_objects.at(i).object.shape, marker.points))
        continue;
      marker.scale.x = 0.02;
    }
    else if (input_msg->feature_objects.at(i).object.shape.type == autoware_msgs::Shape::CYLINDER)
    {
      marker.type = visualization_msgs::Marker::LINE_LIST;
      if (!calcCylinderLineList(input_msg->feature_objects.at(i).object.shape, marker.points))
        continue;
      marker.scale.x = 0.02;
    }
    else if (input_msg->feature_objects.at(i).object.shape.type == autoware_msgs::Shape::POLYGON)
    {
      marker.type = visualization_msgs::Marker::LINE_LIST;
      if (!calcPolygonLineList(input_msg->feature_objects.at(i).object.shape, marker.points))
        continue;
      marker.scale.x = 0.02;
    }
    else
    {
      marker.type = visualization_msgs::Marker::LINE_LIST;
      if (!calcPolygonLineList(input_msg->feature_objects.at(i).object.shape, marker.points))
        continue;
      marker.scale.x = 0.02;
    }

    marker.action = visualization_msgs::Marker::MODIFY;
    marker.pose = input_msg->feature_objects.at(i).object.state.pose.pose;
    marker.lifetime = ros::Duration(0.5);
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    output.markers.push_back(marker);
  }

  // orientation
  for (size_t i = 0; i < input_msg->feature_objects.size(); ++i)
  {
    if (!input_msg->feature_objects.at(i).object.state.pose_reliable)
    {
      continue;
    }
    if (only_known_objects_)
    {
      if (input_msg->feature_objects.at(i).object.semantic.type == autoware_msgs::Semantic::UNKNOWN)
        continue;
    }
    visualization_msgs::Marker marker;
    marker.header = input_msg->header;
    marker.id = id++;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.ns = std::string("orientation");
    marker.scale.x = 0.02;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.pose = input_msg->feature_objects.at(i).object.state.pose.pose;
    geometry_msgs::Point point;
    point.x = 0.0;
    point.y = 0;
    point.z = 0;
    marker.points.push_back(point);
    point.x = (input_msg->feature_objects.at(i).object.shape.dimensions.x/ 2.0);
    point.y = 0;
    point.z = 0;
    marker.points.push_back(point);

    marker.lifetime = ros::Duration(0.5);
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    output.markers.push_back(marker);
  }

  // label
  for (size_t i = 0; i < input_msg->feature_objects.size(); ++i)
  {
    if (only_known_objects_)
    {
      if (input_msg->feature_objects.at(i).object.semantic.type == autoware_msgs::Semantic::UNKNOWN)
        continue;
    }
    visualization_msgs::Marker marker;
    marker.header = input_msg->header;
    marker.id = id++;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.ns = std::string("label");
    std::string label;
    if (!getLabel(input_msg->feature_objects.at(i).object.semantic, label))
      continue;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.text = label;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.pose = input_msg->feature_objects.at(i).object.state.pose.pose;
    marker.pose.position.z = 0.0;
    marker.lifetime = ros::Duration(0.5);
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    output.markers.push_back(marker);
  }

  pub_.publish(output);
}

void DynamicObjectVisualizer::dynamicObjectCallback(const autoware_msgs::DynamicObjectArray::ConstPtr &input_msg)
{
  visualization_msgs::MarkerArray output;
  pub_.publish(output);
}

bool DynamicObjectVisualizer::calcBoundingBoxLineList(const autoware_msgs::Shape &shape, std::vector<geometry_msgs::Point> &points)
{
  geometry_msgs::Point point;
  point.x = shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = -shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = -shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  // up surface
  point.x = shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = -shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);

  // down surface
  point.x = shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = -shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  return true;
}

bool DynamicObjectVisualizer::calcCylinderLineList(const autoware_msgs::Shape &shape, std::vector<geometry_msgs::Point> &points)
{
  int n = 20;
  for (int i = 0; i < n; ++i)
  {
    geometry_msgs::Point point;
    point.x = std::cos(((double)i / (double)n) * 2.0 * M_PI + M_PI / (double)n) * (shape.dimensions.x / 2.0);
    point.y = std::sin(((double)i / (double)n) * 2.0 * M_PI + M_PI / (double)n) * (shape.dimensions.x / 2.0);
    point.z = shape.dimensions.z / 2.0;
    points.push_back(point);
    point.x = std::cos(((double)(i + 1.0) / (double)n) * 2.0 * M_PI + M_PI / (double)n) * (shape.dimensions.x / 2.0);
    point.y = std::sin(((double)(i + 1.0) / (double)n) * 2.0 * M_PI + M_PI / (double)n) * (shape.dimensions.x / 2.0);
    point.z = shape.dimensions.z / 2.0;
    points.push_back(point);
  }
  for (int i = 0; i < n; ++i)
  {
    geometry_msgs::Point point;
    point.x = std::cos(((double)i / (double)n) * 2.0 * M_PI + M_PI / (double)n) * (shape.dimensions.x / 2.0);
    point.y = std::sin(((double)i / (double)n) * 2.0 * M_PI + M_PI / (double)n) * (shape.dimensions.x / 2.0);
    point.z = -shape.dimensions.z / 2.0;
    points.push_back(point);
    point.x = std::cos(((double)(i + 1.0) / (double)n) * 2.0 * M_PI + M_PI / (double)n) * (shape.dimensions.x / 2.0);
    point.y = std::sin(((double)(i + 1.0) / (double)n) * 2.0 * M_PI + M_PI / (double)n) * (shape.dimensions.x / 2.0);
    point.z = -shape.dimensions.z / 2.0;
    points.push_back(point);
  }
  for (int i = 0; i < n; ++i)
  {
    geometry_msgs::Point point;
    point.x = std::cos(((double)i / (double)n) * 2.0 * M_PI + M_PI / (double)n) * (shape.dimensions.x / 2.0);
    point.y = std::sin(((double)i / (double)n) * 2.0 * M_PI + M_PI / (double)n) * (shape.dimensions.x / 2.0);
    point.z = shape.dimensions.z / 2.0;
    points.push_back(point);
    point.x = std::cos(((double)i / (double)n) * 2.0 * M_PI + M_PI / (double)n) * (shape.dimensions.x / 2.0);
    point.y = std::sin(((double)i / (double)n) * 2.0 * M_PI + M_PI / (double)n) * (shape.dimensions.x / 2.0);
    point.z = -shape.dimensions.z / 2.0;
    points.push_back(point);
  }
  return true;
}

bool DynamicObjectVisualizer::calcPolygonLineList(const autoware_msgs::Shape &shape, std::vector<geometry_msgs::Point> &points)
{
  if (shape.footprint.points.size() < 2)
    return false;
  for (size_t i = 0; i < shape.footprint.points.size(); ++i)
  {
    geometry_msgs::Point point;
    point.x = shape.footprint.points.at(i).x;
    point.y = shape.footprint.points.at(i).y;
    point.z = shape.dimensions.z / 2.0;
    points.push_back(point);
    point.x = shape.footprint.points.at((int)(i + 1) % (int)shape.footprint.points.size()).x;
    point.y = shape.footprint.points.at((int)(i + 1) % (int)shape.footprint.points.size()).y;
    point.z = shape.dimensions.z / 2.0;
    points.push_back(point);
  }
  for (size_t i = 0; i < shape.footprint.points.size(); ++i)
  {
    geometry_msgs::Point point;
    point.x = shape.footprint.points.at(i).x;
    point.y = shape.footprint.points.at(i).y;
    point.z = -shape.dimensions.z / 2.0;
    points.push_back(point);
    point.x = shape.footprint.points.at((int)(i + 1) % (int)shape.footprint.points.size()).x;
    point.y = shape.footprint.points.at((int)(i + 1) % (int)shape.footprint.points.size()).y;
    point.z = -shape.dimensions.z / 2.0;
    points.push_back(point);
  }
  for (size_t i = 0; i < shape.footprint.points.size(); ++i)
  {
    geometry_msgs::Point point;
    point.x = shape.footprint.points.at(i).x;
    point.y = shape.footprint.points.at(i).y;
    point.z = shape.dimensions.z / 2.0;
    points.push_back(point);
    point.x = shape.footprint.points.at(i).x;
    point.y = shape.footprint.points.at(i).y;
    point.z = -shape.dimensions.z / 2.0;
    points.push_back(point);
  }
  return true;
}
bool DynamicObjectVisualizer::getLabel(const autoware_msgs::Semantic &semantic, std::string &label)
{

  if (autoware_msgs::Semantic::UNKNOWN == semantic.type)
  {
    label = std::string("unknown");
  }
  else if (autoware_msgs::Semantic::CAR == semantic.type)
  {
    label = std::string("car");
  }
  else if (autoware_msgs::Semantic::TRUCK == semantic.type)
  {
    label = std::string("truck");
  }
  else if (autoware_msgs::Semantic::BUS == semantic.type)
  {
    label = std::string("bus");
  }
  else if (autoware_msgs::Semantic::PEDESTRIAN == semantic.type)
  {
    label = std::string("pedestrian");
  }
  else
  {
    label = std::string("unknown");
  }
  return true;
}