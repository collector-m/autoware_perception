/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * v1.0 Yukihiro Saito
 */

#include "multi_object_tracker/node.hpp"
#include "multi_object_tracker/tracker/tracker.hpp"
#include "multi_object_tracker/data_association/data_association.hpp"
#include <string>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

MultiObjectTrackerNode::MultiObjectTrackerNode() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
    sub_ = nh_.subscribe("input", 1, &MultiObjectTrackerNode::measurementCallback, this);
    pub_ = nh_.advertise<autoware_msgs::DynamicObjectArray>("output", 1, true);
    double publish_rate;
    pnh_.param<double>("publish_rate", publish_rate, double(10.0));
    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate), &MultiObjectTrackerNode::publishTimerCallback, this);
    pnh_.param<std::string>("world_frame_id", world_frame_id_, std::string("world"));
}

void MultiObjectTrackerNode::measurementCallback(const autoware_msgs::DynamicObjectWithFeatureArray::ConstPtr &input_msg)
{
    if (input_msg->header.frame_id != world_frame_id_)
    {
        geometry_msgs::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_.lookupTransform(/*target*/ world_frame_id_, /*src*/ input_msg->header.frame_id,
                                                           input_msg->header.stamp);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }
    }

    /* tracker prediction */
    ros::Time current_time = ros::Time::now();
    for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr)
    {
        (*itr)->predict(current_time);
    }

    /* life cycle check */
    // TODO

    /* global nearest neighboor */
    std::unordered_map<int, int> direct_assignment;
    std::unordered_map<int, int> reverse_assignment;
    Eigen::MatrixXd score_matrix = data_association_.calcScoreMatrix(*input_msg, list_tracker_); // row : tracker, col : measurement
    data_association_.assign(score_matrix, direct_assignment, reverse_assignment);

    /* tracker measurement update */
    int tracker_idx = 0;
    for (auto tracker_itr = list_tracker_.begin(); tracker_itr != list_tracker_.end(); ++tracker_itr, ++tracker_idx)
    {
        if (direct_assignment.find(tracker_idx) != direct_assignment.end()) // found
        {
            (*(tracker_itr))->updateWithMeasurement(input_msg->feature_objects.at(direct_assignment.find(tracker_idx)->second).object);
        }
        else
        {
            (*(tracker_itr))->updateWithoutMeasurement();
        }
    }

    /* new tracker */
    for (size_t i = 0; i < input_msg->feature_objects.size(); ++i)
    {
        if (reverse_assignment.find(i) != reverse_assignment.end()) // found
            continue;

        if (input_msg->feature_objects.at(i).object.semantic.type == autoware_msgs::Semantic::CAR || input_msg->feature_objects.at(i).object.semantic.type == autoware_msgs::Semantic::TRUCK || input_msg->feature_objects.at(i).object.semantic.type == autoware_msgs::Semantic::BUS)
        {
            list_tracker_.push_back(std::make_shared<VehicleTracker>(input_msg->feature_objects.at(i).object));
        }
        else
        {
            // list_tracker_.push_back(std::make_shared<VehicleTracker>(input_msg->feature_objects.at(i).object));
        }
    }
}

void MultiObjectTrackerNode::publishTimerCallback(const ros::TimerEvent &e)
{
    // Guard
    if (pub_.getNumSubscribers() < 1)
        return;

    /* tracker prediction */
    ros::Time current_time = ros::Time::now();
    for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr)
    {
        (*itr)->predict(current_time);
    }

    /* life cycle check */
    for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr)
    {
        if (1.0 < (*itr)->getElapsedTimeFromLastUpdate())
        {
            auto erase_itr = itr;
            --itr;
            list_tracker_.erase(erase_itr);
            std::cout << "detele" << std::endl;
        }
    }

    // Create output msg
    autoware_msgs::DynamicObjectArray output_msg;
    output_msg.header.frame_id = world_frame_id_;
    output_msg.header.frame_id = "velodyne"; // TODO
    output_msg.header.stamp = current_time;
    for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr)
    {
        if ((*itr)->getTotalMeasurementCount() < 3)
            continue;
        autoware_msgs::DynamicObject object;
        (*itr)->getEstimatedDynamicObject(object);
        output_msg.objects.push_back(object);
    }

    // Publish
    pub_.publish(output_msg);
    return;
}