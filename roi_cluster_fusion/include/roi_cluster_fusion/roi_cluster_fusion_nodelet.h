#pragma once
#include "ros/ros.h"

#include "nodelet/nodelet.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/transform_datatypes.h>
// #include <tf2/convert.h>
// #include <tf2/LinearMath/Transform.h>
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "sensor_msgs/CameraInfo.h"
#include "autoware_msgs/DynamicObjectWithFeatureArray.h"
#include "autoware_msgs/DynamicObjectWithFeature.h"
#include <memory>

namespace roi_cluster_fusion
{
class RoiClusterFusionNodelet : public nodelet::Nodelet
{

public:
  RoiClusterFusionNodelet();

private:
  virtual void onInit();

  void fusionCallback(const autoware_msgs::DynamicObjectWithFeatureArrayConstPtr &input_cluster_msg,
                      const autoware_msgs::DynamicObjectWithFeatureArrayConstPtr &input_roi_msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &input_camera_info_msg);
  double calcIoU(const sensor_msgs::RegionOfInterest &roi_1,
                 const sensor_msgs::RegionOfInterest &roi_2);
  double calcIoUX(const sensor_msgs::RegionOfInterest &roi_1,
                  const sensor_msgs::RegionOfInterest &roi_2);
  double calcIoUY(const sensor_msgs::RegionOfInterest &roi_1,
                  const sensor_msgs::RegionOfInterest &roi_2);

  ros::NodeHandle nh_, private_nh_;
  ros::Publisher labeled_cluster_pub_;
  ros::Subscriber camera_info_sub_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
  message_filters::Subscriber<autoware_msgs::DynamicObjectWithFeatureArray> cluster_sub_;
  message_filters::Subscriber<autoware_msgs::DynamicObjectWithFeatureArray> roi_sub_;
  // message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub_;

  typedef message_filters::sync_policies::ApproximateTime<autoware_msgs::DynamicObjectWithFeatureArray,
                                                          autoware_msgs::DynamicObjectWithFeatureArray>
      SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_ptr_;
  std::shared_ptr<sensor_msgs::CameraInfo> camera_info_ptr_;
  // ROS Parameters
  bool use_iou_x_;
  bool use_iou_y_;
  bool use_iou_;
  double iou_threshold_;
};

} // namespace roi_cluster_fusion
