#include <roi_cluster_fusion/roi_cluster_fusion_nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <map>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace roi_cluster_fusion
{

RoiClusterFusionNodelet::RoiClusterFusionNodelet() {}

void RoiClusterFusionNodelet::onInit()
{
    private_nh_ = getPrivateNodeHandle();

    private_nh_.param<bool>("use_iou_x_", use_iou_x_, true);
    private_nh_.param<bool>("use_iou_y_", use_iou_y_, false);
    private_nh_.param<bool>("use_iou_", use_iou_, false);
    private_nh_.param<double>("iou_threshold", iou_threshold_, 0.1);

    nh_ = getNodeHandle();
    tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
    cluster_sub_.subscribe(nh_, "clusters", 1);
    roi_sub_.subscribe(nh_, "rois", 1);
    camera_info_sub_ = nh_.subscribe("camera_info", 1, &RoiClusterFusionNodelet::cameraInfoCallback, this);

    sync_ptr_ = std::make_shared<Sync>(SyncPolicy(10), cluster_sub_, roi_sub_);
    sync_ptr_->registerCallback(boost::bind(&RoiClusterFusionNodelet::fusionCallback, this, _1, _2));

    labeled_cluster_pub_ = nh_.advertise<autoware_msgs::DynamicObjectWithFeatureArray>("labeled_clusters", 10);
}

void RoiClusterFusionNodelet::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &input_camera_info_msg)
{
    camera_info_ptr_ = std::make_shared<sensor_msgs::CameraInfo>(*input_camera_info_msg);
}

void RoiClusterFusionNodelet::fusionCallback(const autoware_msgs::DynamicObjectWithFeatureArrayConstPtr &input_cluster_msg,
                                             const autoware_msgs::DynamicObjectWithFeatureArrayConstPtr &input_roi_msg)
{
    if (camera_info_ptr_ == nullptr)
    {
        ROS_WARN("no camera info");
        return;
    }
    // build output msg
    autoware_msgs::DynamicObjectWithFeatureArray output_msg;
    output_msg = *input_cluster_msg;
    // intrinsic
    Eigen::Matrix3d intrinsic;
    intrinsic << camera_info_ptr_->K.at(0),
        camera_info_ptr_->K.at(1),
        camera_info_ptr_->K.at(2),
        camera_info_ptr_->K.at(3),
        camera_info_ptr_->K.at(4),
        camera_info_ptr_->K.at(5),
        camera_info_ptr_->K.at(6),
        camera_info_ptr_->K.at(7),
        camera_info_ptr_->K.at(8);

    // affine matrix
    Eigen::Affine3d affine_mat;
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        transform_stamped = tf_buffer_.lookupTransform(/*target*/ camera_info_ptr_->header.frame_id, /*src*/ input_cluster_msg->header.frame_id,
                                                       ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
    Eigen::Translation<double, 3> trans = Eigen::Translation<double, 3>(transform_stamped.transform.translation.x,
                                                                        transform_stamped.transform.translation.y,
                                                                        transform_stamped.transform.translation.z);
    Eigen::Quaterniond quat(transform_stamped.transform.rotation.w, transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z);
    affine_mat = trans * quat;

    // build cluster roi
    std::map<size_t, sensor_msgs::RegionOfInterest> m_cluster_roi;
    for (size_t i = 0; i < input_cluster_msg->feature_objects.size(); ++i)
    {
        if (input_cluster_msg->feature_objects.at(i).feature.cluster.data.empty())
            continue;
        std::vector<Eigen::Vector2d> projected_points;
        for (sensor_msgs::PointCloud2ConstIterator<float>
                 iter_x(input_cluster_msg->feature_objects.at(i).feature.cluster, "x"),
             iter_y(input_cluster_msg->feature_objects.at(i).feature.cluster, "y"),
             iter_z(input_cluster_msg->feature_objects.at(i).feature.cluster, "z");
             iter_x != iter_x.end();
             ++iter_x, ++iter_y, ++iter_z)
        {
            Eigen::Vector3d transformed_point = (affine_mat * Eigen::Vector3d(*iter_x, *iter_y, *iter_z));
            if (transformed_point.z() <= 0.0)
                continue;
            Eigen::Vector3d projected_point = intrinsic * transformed_point;
            projected_points.push_back(Eigen::Vector2d(projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z()));
        }
        if (projected_points.empty())
            continue;

        bool in_image = false;
        int min_x(camera_info_ptr_->width), min_y(camera_info_ptr_->height), max_x(0), max_y(0);
        for (const auto &projected_point : projected_points)
        {
            if (0 <= (int)projected_point.x() &&
                (int)projected_point.x() <= (int)camera_info_ptr_->width - 1 &&
                0 <= (int)projected_point.y() &&
                (int)projected_point.y() <= (int)camera_info_ptr_->height - 1)
            {
                in_image = true;
                if (projected_point.x() <= min_x)
                    min_x = std::max((int)projected_point.x(), 0);
                if (projected_point.y() <= min_y)
                    min_y = std::max((int)projected_point.y(), 0);
                if (max_x <= projected_point.x())
                    max_x = std::min((int)projected_point.x(), (int)camera_info_ptr_->width - 1);
                if (max_y <= projected_point.y())
                    max_y = std::min((int)projected_point.y(), (int)camera_info_ptr_->height - 1);
            }
        }
        sensor_msgs::RegionOfInterest roi;
        // roi.do_rectify = camera_info_ptr_->do_rectify;
        roi.x_offset = min_x;
        roi.y_offset = min_y;
        roi.width = max_x - min_x;
        roi.height = max_y - min_y;
        if (in_image)
            m_cluster_roi.insert(std::make_pair(i, roi));
    }

    // calc iou
    for (size_t i = 0; i < input_roi_msg->feature_objects.size(); ++i)
    {
        int index;
        double max_iou = 0.0;
        for (auto m_cluster_roi_itr = m_cluster_roi.begin(); m_cluster_roi_itr != m_cluster_roi.end(); ++m_cluster_roi_itr)
        {
            double iou(0.0), iou_x(0.0), iou_y(0.0);
            if (use_iou_)
                iou = calcIoU(m_cluster_roi_itr->second, input_roi_msg->feature_objects.at(i).feature.roi);
            if (use_iou_x_)
                iou_x = calcIoUX(m_cluster_roi_itr->second, input_roi_msg->feature_objects.at(i).feature.roi);
            if (use_iou_y_)
                iou_y = calcIoUY(m_cluster_roi_itr->second, input_roi_msg->feature_objects.at(i).feature.roi);
            if (max_iou < iou + iou_x + iou_y)
            {
                index = m_cluster_roi_itr->first;
                max_iou = iou + iou_x + iou_y;
            }
        }
        if (iou_threshold_ < max_iou)
            output_msg.feature_objects.at(index).object.semantic = input_roi_msg->feature_objects.at(i).object.semantic;
    }

    // publish output msg
    labeled_cluster_pub_.publish(output_msg);

#if 0
    cv::namedWindow("Sample", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);

    cv::Mat image(cv::Size(camera_info_ptr_->width, camera_info_ptr_->height), CV_8UC3, cv::Scalar(0, 0, 0));

    for (size_t i = 0; i < input_cluster_msg->feature_objects.size(); ++i)
    {
        if (input_cluster_msg->feature_objects.at(i).feature.cluster.data.empty())
            continue;
        std::vector<Eigen::Vector2d> projected_points;
        for (sensor_msgs::PointCloud2ConstIterator<float>
                 iter_x(input_cluster_msg->feature_objects.at(i).feature.cluster, "x"),
             iter_y(input_cluster_msg->feature_objects.at(i).feature.cluster, "y"),
             iter_z(input_cluster_msg->feature_objects.at(i).feature.cluster, "z");
             iter_x != iter_x.end();
             ++iter_x, ++iter_y, ++iter_z)
        {
            Eigen::Vector3d transformed_point = (affine_mat * Eigen::Vector3d(*iter_x, *iter_y, *iter_z));
            if (transformed_point.z() <= 0.0)
                continue;
            Eigen::Vector3d projected_point = intrinsic * transformed_point;
            projected_points.push_back(Eigen::Vector2d(projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z()));
        }
        if (projected_points.empty())
            continue;

        bool in_image = false;
        int min_x(camera_info_ptr_->width), min_y(camera_info_ptr_->height), max_x(0), max_y(0);
        for (const auto &projected_point : projected_points)
        {
            if (0 <= (int)projected_point.x() &&
                (int)projected_point.x() <= (int)camera_info_ptr_->width - 1 &&
                0 <= (int)projected_point.y() &&
                (int)projected_point.y() <= (int)camera_info_ptr_->height - 1)
            {
                cv::circle(image, cv::Point((int)projected_point.x(), (int)projected_point.y()), 2, cv::Scalar(255, 255, 255), 3, 4);

                in_image = true;
                if (projected_point.x() <= min_x)
                    min_x = std::max((int)projected_point.x(), 0);
                if (projected_point.y() <= min_y)
                    min_y = std::max((int)projected_point.y(), 0);
                if (max_x <= projected_point.x())
                    max_x = std::min((int)projected_point.x(), (int)camera_info_ptr_->width - 1);
                if (max_y <= projected_point.y())
                    max_y = std::min((int)projected_point.y(), (int)camera_info_ptr_->height - 1);
            }
        }

        if (in_image)
        {
            cv::line(image,
                     cv::Point(min_x, min_y),
                     cv::Point(max_x, min_y),
                     cv::Scalar(0, 255, 255), 1, CV_AA);
            cv::line(image,
                     cv::Point(min_x, min_y),
                     cv::Point(min_x, max_y),
                     cv::Scalar(0, 255, 255), 1, CV_AA);
            cv::line(image,
                     cv::Point(min_x, max_y),
                     cv::Point(max_x, max_y),
                     cv::Scalar(0, 255, 255), 1, CV_AA);
            cv::line(image,
                     cv::Point(max_x, min_y),
                     cv::Point(max_x, max_y),
                     cv::Scalar(0, 255, 255), 1, CV_AA);
        }
    }

    for (size_t i = 0; i < input_roi_msg->feature_objects.size(); ++i)
    {
        cv::line(image,
                 cv::Point(input_roi_msg->feature_objects.at(i).feature.roi.x_offset, input_roi_msg->feature_objects.at(i).feature.roi.y_offset),
                 cv::Point(input_roi_msg->feature_objects.at(i).feature.roi.x_offset + input_roi_msg->feature_objects.at(i).feature.roi.width, input_roi_msg->feature_objects.at(i).feature.roi.y_offset),
                 cv::Scalar(0, 255, 0), 3, CV_AA);
        cv::line(image,
                 cv::Point(input_roi_msg->feature_objects.at(i).feature.roi.x_offset, input_roi_msg->feature_objects.at(i).feature.roi.y_offset),
                 cv::Point(input_roi_msg->feature_objects.at(i).feature.roi.x_offset, input_roi_msg->feature_objects.at(i).feature.roi.y_offset + input_roi_msg->feature_objects.at(i).feature.roi.height),
                 cv::Scalar(0, 255, 0), 3, CV_AA);
        cv::line(image,
                 cv::Point(input_roi_msg->feature_objects.at(i).feature.roi.x_offset + input_roi_msg->feature_objects.at(i).feature.roi.width, input_roi_msg->feature_objects.at(i).feature.roi.y_offset),
                 cv::Point(input_roi_msg->feature_objects.at(i).feature.roi.x_offset + input_roi_msg->feature_objects.at(i).feature.roi.width, input_roi_msg->feature_objects.at(i).feature.roi.y_offset + input_roi_msg->feature_objects.at(i).feature.roi.height),
                 cv::Scalar(0, 255, 0), 3, CV_AA);
        cv::line(image,
                 cv::Point(input_roi_msg->feature_objects.at(i).feature.roi.x_offset, input_roi_msg->feature_objects.at(i).feature.roi.y_offset + input_roi_msg->feature_objects.at(i).feature.roi.height),
                 cv::Point(input_roi_msg->feature_objects.at(i).feature.roi.x_offset + input_roi_msg->feature_objects.at(i).feature.roi.width, input_roi_msg->feature_objects.at(i).feature.roi.y_offset + input_roi_msg->feature_objects.at(i).feature.roi.height),
                 cv::Scalar(0, 255, 0), 3, CV_AA);
    }
    cv::imshow("Sample", image);
    cv::waitKey(3);
#endif
}

double RoiClusterFusionNodelet::calcIoU(const sensor_msgs::RegionOfInterest &roi_1,
                                        const sensor_msgs::RegionOfInterest &roi_2)
{
    double s_1, s_2;
    s_1 = (double)roi_1.width * (double)roi_1.height;
    s_2 = (double)roi_2.width * (double)roi_2.height;

    double overlap_s;
    double overlap_max_x, overlap_max_y, overlap_min_x, overlap_min_y;
    overlap_min_x = roi_1.x_offset < roi_2.x_offset ? roi_2.x_offset : roi_1.x_offset;
    overlap_min_y = roi_1.y_offset < roi_2.y_offset ? roi_2.y_offset : roi_1.y_offset;
    overlap_max_x = roi_1.x_offset + roi_1.width < roi_2.x_offset + roi_2.width ? roi_1.x_offset + roi_1.width : roi_2.x_offset + roi_2.width;
    overlap_max_y = roi_1.y_offset + roi_1.height < roi_2.y_offset + roi_2.height ? roi_1.y_offset + roi_1.height : roi_2.y_offset + roi_2.height;
    overlap_s = (overlap_max_x - overlap_min_x) * (overlap_max_y - overlap_min_y);
    if (overlap_max_x < overlap_min_x || overlap_max_y < overlap_min_y)
        return 0.0;
    return overlap_s / (s_1 + s_2 - overlap_s);
}
double RoiClusterFusionNodelet::calcIoUX(const sensor_msgs::RegionOfInterest &roi_1,
                                         const sensor_msgs::RegionOfInterest &roi_2)
{
    double s_1, s_2;
    s_1 = (double)roi_1.width;
    s_2 = (double)roi_2.width;
    double overlap_s;
    double overlap_max_x, overlap_max_y, overlap_min_x, overlap_min_y;
    overlap_min_x = roi_1.x_offset < roi_2.x_offset ? roi_2.x_offset : roi_1.x_offset;
    overlap_min_y = roi_1.y_offset < roi_2.y_offset ? roi_2.y_offset : roi_1.y_offset;
    overlap_max_x = roi_1.x_offset + roi_1.width < roi_2.x_offset + roi_2.width ? roi_1.x_offset + roi_1.width : roi_2.x_offset + roi_2.width;
    overlap_max_y = roi_1.y_offset + roi_1.height < roi_2.y_offset + roi_2.height ? roi_1.y_offset + roi_1.height : roi_2.y_offset + roi_2.height;
    overlap_s = (overlap_max_x - overlap_min_x);
    if (overlap_max_x < overlap_min_x || overlap_max_y < overlap_min_y)
        return 0.0;
    return overlap_s / (s_1 + s_2 - overlap_s);
}
double RoiClusterFusionNodelet::calcIoUY(const sensor_msgs::RegionOfInterest &roi_1,
                                         const sensor_msgs::RegionOfInterest &roi_2)
{
    double s_1, s_2;
    s_1 = (double)roi_1.height;
    s_2 = (double)roi_2.height;
    double overlap_s;
    double overlap_max_x, overlap_max_y, overlap_min_x, overlap_min_y;
    overlap_min_x = roi_1.x_offset < roi_2.x_offset ? roi_2.x_offset : roi_1.x_offset;
    overlap_min_y = roi_1.y_offset < roi_2.y_offset ? roi_2.y_offset : roi_1.y_offset;
    overlap_max_x = roi_1.x_offset + roi_1.width < roi_2.x_offset + roi_2.width ? roi_1.x_offset + roi_1.width : roi_2.x_offset + roi_2.width;
    overlap_max_y = roi_1.y_offset + roi_1.height < roi_2.y_offset + roi_2.height ? roi_1.y_offset + roi_1.height : roi_2.y_offset + roi_2.height;
    overlap_s = (overlap_max_y - overlap_min_y);
    if (overlap_max_x < overlap_min_x || overlap_max_y < overlap_min_y)
        return 0.0;
    return overlap_s / (s_1 + s_2 - overlap_s);
}
} // namespace roi_cluster_fusion

PLUGINLIB_EXPORT_CLASS(roi_cluster_fusion::RoiClusterFusionNodelet, nodelet::Nodelet)