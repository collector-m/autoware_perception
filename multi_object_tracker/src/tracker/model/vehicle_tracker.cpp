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

#include "multi_object_tracker/tracker/model/vehicle_tracker.hpp"

VehicleTracker::VehicleTracker(const autoware_msgs::DynamicObject &object)
    : Tracker(object.semantic.type)
{
    object_ = object;
}

bool VehicleTracker::predict(const ros::Time &time)
{
    return true;
}
bool VehicleTracker::measure(const autoware_msgs::DynamicObject &object)
{
    int type = object.semantic.type;
    if (type == autoware_msgs::Semantic::UNKNOWN)
        type = object_.semantic.type;
    object_ = object;
    object_.semantic.type = type;

    


    return true;
}

bool VehicleTracker::getEstimatedDynamicObject(autoware_msgs::DynamicObject &object)
{
    object = object_;
    object.id = unique_id::toMsg(uuid_);
    return true;
}

geometry_msgs::Point VehicleTracker::getPosition()
{
    geometry_msgs::Point position;
    position.x = object_.state.pose.pose.position.x;
    position.y = object_.state.pose.pose.position.y;
    position.z = object_.state.pose.pose.position.z;
    return position;
}