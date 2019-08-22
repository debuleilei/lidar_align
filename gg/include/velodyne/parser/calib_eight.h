/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include<string>  
#include<iostream>  
#include<unordered_map>

#include <Eigen/Eigen>

#include "cybertron/tf2_cybertron/transform_listener.h"
#include "cybertron/tf2_cybertron/transform_broadcaster.h"
#include "cybertron/time/time.h"
#include "proto/sensor_pointcloud.pb.h"
#include "proto/novatel_ins.pb.h"
#include "proto/velodyne_config.pb.h"

#include "velodyne/lib/const_variables.h"


namespace cybertron {
namespace drivers {
namespace velodyne {

class Calibrate_eight {

 public:
   explicit Calibrate_eight(cybertron::proto::VelodyneConfig& velodyne_config);
  virtual ~Calibrate_eight() {}

  bool calib_eight_back(
  const std::shared_ptr < adu::common::sensor::PointCloud const>& point_cloud_first,
  const std::shared_ptr < adu::common::sensor::INSPVA   const>& INSPVA,
  std::shared_ptr< adu::common::sensor::PointCloud>& point_cloud_fusion);

  bool calib_eight_left(
  const std::shared_ptr < adu::common::sensor::PointCloud const>& point_cloud_first,
  const std::shared_ptr < adu::common::sensor::INSPVA   const>& INSPVA,
  std::shared_ptr< adu::common::sensor::PointCloud>& point_cloud_fusion);

  bool calib_eight_right(
  const std::shared_ptr < adu::common::sensor::PointCloud const>& point_cloud_first,
  const std::shared_ptr < adu::common::sensor::INSPVA   const>& INSPVA,
  std::shared_ptr< adu::common::sensor::PointCloud>& point_cloud_fusion);

  bool calib_eight_lidar(
  const std::shared_ptr < adu::common::sensor::PointCloud const>& point_cloud_first,
  const std::shared_ptr < adu::common::sensor::PointCloud const>& point_cloud_second,
  const std::shared_ptr < adu::common::sensor::INSPVA   const>& INSPVA,
  std::shared_ptr< adu::common::sensor::PointCloud>& point_cloud_fusion);

  bool velodyne_add(const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_left,
                  const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_right,
                  const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_back,
                  std::shared_ptr<adu::common::sensor::PointCloud>& point_cloud_fusion);


  cybertron::proto::VelodyneConfig _config;
};


}  // namespace velodyne
}
}

  
