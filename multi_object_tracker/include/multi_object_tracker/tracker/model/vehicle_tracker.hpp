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
 * v1.0 Yukihiro Saito
 */

#pragma once
#include "tracker_base.hpp"
#include "autoware_msgs/DynamicObject.h"

class VehicleTracker : public Tracker
{
  private:
    autoware_msgs::DynamicObject object_;

  public:
    VehicleTracker(const autoware_msgs::DynamicObject &object);

    bool predict(const ros::Time &time) override;
    bool measure(const autoware_msgs::DynamicObject &object) override;
    bool getEstimatedDynamicObject(autoware_msgs::DynamicObject &object) override;
    geometry_msgs::Point getPosition() override;

    virtual ~VehicleTracker(){};
};
