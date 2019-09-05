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

#ifndef ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_COMPONENT_COMPONENT_CALIB_EIGHT_H
#define ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_COMPONENT_COMPONENT_CALIB_EIGHT_H


#include "cybertron/common/common.h"
#include "cybertron/dag_streaming/component.h"
#include "cybertron/time/time.h"
#include "proto/sensor_velodyne.pb.h"
#include "proto/sensor_pointcloud.pb.h"
// #include"proto/novatel_bestpos.pb.h"
// #include"proto/novatel_heading.pb.h"
#include "proto/velodyne_config.pb.h"      //tofind
#include "proto/novatel_ins.pb.h"
#include <yaml-cpp/yaml.h>

#include "velodyne/parser/calib_eight.h"  


namespace cybertron {
namespace drivers {
namespace velodyne {


class Comptransform_pointcloud : public Component<adu::common::sensor::PointCloud> {
 public:
  int Init() override {
    std::cout<<"start lalalalalalal!";
     cybertron::proto::VelodyneConfig velodyne_config;
    // if (ParseConfig(&velodyne_config) != cybertron::SUCC) {
    //   LOG_ERROR << DRIVER_ERROR << LIDAR_CONF_PARSE_ERR << " Parse config file error";
    //   return cybertron::FAIL;
    // }

    std::string config_file = _conf;
    std::string content;   

    if(!cybertron::FileUtil::get_file_content(config_file, &content)){
      LOG_ERROR<<"failed to load config file!";
      return FAIL;
    }
    std::cout<<"content:"<<content<<endl;
    if(!google::protobuf::TextFormat::ParseFromString(content, &velodyne_config)){
      LOG_ERROR<<"failed to parser suteng config config file!";
      return FAIL;
    }
    std::cout<<"start 62";
    lidar_extrinsic_path_ = velodyne_config.lidars_filter_config_path();
    std::cout<<"start 64";
    LOG_INFO<<"lidar_extrinsic_file_path:"<<lidar_extrinsic_path_;

    bool success = LoadLidarExtrinsic(lidar_extrinsic_path_, &lidar_extrinsic_);
    if(!success){
      LOG_ERROR<<"failed to get lidar extrinsic from file!";
      return FAIL;
    }
    // _fusion.reset(new Calibrate(velodyne_config));

    fusion_deque_.resize(size_);
    for (int i = 0; i < size_; ++i) {
      fusion_deque_[i] = std::make_shared<adu::common::sensor::PointCloud>();
      if (fusion_deque_[i] == nullptr) {
        LOG_ERROR << DRIVER_ERROR << LIDAR_INIT_ERR << " fail to make shared";
        return FAIL;
      }
      
      fusion_deque_[i]->mutable_point()->Reserve(140000);
    }
     cout<<"Comptransform_lidar**************init successfully"<<endl;
    return SUCC;
  }

  int Proc(
      const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_first)override{
    
    if (index_ >= size_) {
      index_ = 0;
    }
    //  cout<<"prockkkkkkkkkkkkk"<<endl;
    auto point_cloud = fusion_deque_.at(index_++);
    point_cloud->Clear();
    if (_fusion->transform_lidar(point_cloud_first, point_cloud,lidar_extrinsic_)) {
      seq_++;
      point_cloud->mutable_header()->set_sequence_num(seq_);
      Send(point_cloud);
      return SUCC;
    } else {
      return FAIL;
    }
 
 }

  bool LoadLidarExtrinsic(const std::string& file_path,
                                         Eigen::Affine3d* lidar_extrinsic) {
  YAML::Node config = YAML::LoadFile(file_path);
  if (config["transform"]) {
    if (config["transform"]["translation"]) {
      lidar_extrinsic->translation()(0) =
          config["transform"]["translation"]["x"].as<double>();
      lidar_extrinsic->translation()(1) =
          config["transform"]["translation"]["y"].as<double>();
      lidar_extrinsic->translation()(2) =
          config["transform"]["translation"]["z"].as<double>();
      if (config["transform"]["rotation"]) {
        double qx = config["transform"]["rotation"]["x"].as<double>();
        double qy = config["transform"]["rotation"]["y"].as<double>();
        double qz = config["transform"]["rotation"]["z"].as<double>();
        double qw = config["transform"]["rotation"]["w"].as<double>();
        lidar_extrinsic->linear() =
            Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
        return true;
      }
    }
  }
  return false;
}

 private:
  std::unique_ptr<Calibrate_eight> _fusion;
  std::deque<std::shared_ptr<adu::common::sensor::PointCloud>> fusion_deque_;
  uint64_t index_ = 0;
  uint64_t seq_ = 0;
  uint64_t size_ = 8;
  std::string lidar_extrinsic_path_;
  Eigen::Affine3d lidar_extrinsic_; 
};

CLASS_LOADER_REGISTER_COMPONENT(Comptransform_pointcloud);

class Compeight_back : public Component<adu::common::sensor::PointCloud,
    adu::common::sensor::INSPVA> {
 public:
  int Init() override {
     cybertron::proto::VelodyneConfig velodyne_config;
    if (ParseConfig(&velodyne_config) != cybertron::SUCC) {
      LOG_ERROR << DRIVER_ERROR << LIDAR_CONF_PARSE_ERR << " Parse config file error";
      return cybertron::FAIL;
    }

    // _fusion.reset(new Calibrate(velodyne_config));

    fusion_deque_.resize(size_);
    for (int i = 0; i < size_; ++i) {
      fusion_deque_[i] = std::make_shared<adu::common::sensor::PointCloud>();
      if (fusion_deque_[i] == nullptr) {
        LOG_ERROR << DRIVER_ERROR << LIDAR_INIT_ERR << " fail to make shared";
        return FAIL;
      }
      
      fusion_deque_[i]->mutable_point()->Reserve(140000);
    }
     cout<<"Compeight_back**************init successfully"<<endl;
    return SUCC;
  }

  int Proc(
      const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_first,
      const std::shared_ptr<adu::common::sensor::INSPVA const>& INSPVA) override{
    
    if (index_ >= size_) {
      index_ = 0;
    }
    //  cout<<"prockkkkkkkkkkkkk"<<endl;
    auto point_cloud = fusion_deque_.at(index_++);
    point_cloud->Clear();
    if (_fusion->calib_eight_back(point_cloud_first, INSPVA, point_cloud)) {
      seq_++;
      point_cloud->mutable_header()->set_sequence_num(seq_);
      Send(point_cloud);
      return SUCC;
    } else {
      return FAIL;
    }
 
 }

 private:
  std::unique_ptr<Calibrate_eight> _fusion;
  std::deque<std::shared_ptr<adu::common::sensor::PointCloud>> fusion_deque_;
  uint64_t index_ = 0;
  uint64_t seq_ = 0;
  uint64_t size_ = 8;
};

CLASS_LOADER_REGISTER_COMPONENT(Compeight_back);

class Compeight_lidar : public Component<adu::common::sensor::PointCloud,
    adu::common::sensor::PointCloud,adu::common::sensor::INSPVA > {
 public:
  int Init() override {
     cybertron::proto::VelodyneConfig velodyne_config;
    if (ParseConfig(&velodyne_config) != cybertron::SUCC) {
      LOG_ERROR << DRIVER_ERROR << LIDAR_CONF_PARSE_ERR << " Parse config file error";
      return cybertron::FAIL;
    }

    // _fusion.reset(new Calibrate(velodyne_config));

    fusion_deque_.resize(size_);
    for (int i = 0; i < size_; ++i) {
      fusion_deque_[i] = std::make_shared<adu::common::sensor::PointCloud>();
      if (fusion_deque_[i] == nullptr) {
        LOG_ERROR << DRIVER_ERROR << LIDAR_INIT_ERR << " fail to make shared";
        return FAIL;
      }
      
      fusion_deque_[i]->mutable_point()->Reserve(140000);
    }
     cout<<"Compeight_lidar**************init successfully"<<endl;
    return SUCC;
  }

  int Proc(
      const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_first,
      const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_second,
      const std::shared_ptr<adu::common::sensor::INSPVA const>& INSPVA ) override{
    
    if (index_ >= size_) {
      index_ = 0;
    }
    //  cout<<"prockkkkkkkkkkkkk"<<endl;
    auto point_cloud = fusion_deque_.at(index_++);
    point_cloud->Clear();
    if (_fusion->calib_eight_lidar(point_cloud_first, point_cloud_second, INSPVA,point_cloud)) {
      seq_++;
      point_cloud->mutable_header()->set_sequence_num(seq_);
      Send(point_cloud);
      return SUCC;
    } else {
      return FAIL;
    }
 
 }

 private:
  std::unique_ptr<Calibrate_eight> _fusion;
  std::deque<std::shared_ptr<adu::common::sensor::PointCloud>> fusion_deque_;
  uint64_t index_ = 0;
  uint64_t seq_ = 0;
  uint64_t size_ = 8;
};

CLASS_LOADER_REGISTER_COMPONENT(Compeight_lidar);

class Compeight_left : public Component<adu::common::sensor::PointCloud,
    adu::common::sensor::INSPVA> {
 public:
  int Init() override {
     cybertron::proto::VelodyneConfig velodyne_config;
    if (ParseConfig(&velodyne_config) != cybertron::SUCC) {
      LOG_ERROR << DRIVER_ERROR << LIDAR_CONF_PARSE_ERR << " Parse config file error";
      return cybertron::FAIL;
    }

    // _fusion.reset(new Calibrate(velodyne_config));

    fusion_deque_.resize(size_);
    for (int i = 0; i < size_; ++i) {
      fusion_deque_[i] = std::make_shared<adu::common::sensor::PointCloud>();
      if (fusion_deque_[i] == nullptr) {
        LOG_ERROR << DRIVER_ERROR << LIDAR_INIT_ERR << " fail to make shared";
        return FAIL;
      }
      
      fusion_deque_[i]->mutable_point()->Reserve(140000);
    }
     cout<<"Compeight_left**************init successfully"<<endl;
    return SUCC;
  }

  int Proc(
      const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_first,
      const std::shared_ptr<adu::common::sensor::INSPVA const>& INSPVA) override{
    
    if (index_ >= size_) {
      index_ = 0;
    }
    //  cout<<"prockkkkkkkkkkkkk"<<endl;
    auto point_cloud = fusion_deque_.at(index_++);
    point_cloud->Clear();
    if (_fusion->calib_eight_left(point_cloud_first, INSPVA, point_cloud)) {
      seq_++;
      point_cloud->mutable_header()->set_sequence_num(seq_);
      Send(point_cloud);
      return SUCC;
    } else {
      return FAIL;
    }
 
 }

 private:
  std::unique_ptr<Calibrate_eight> _fusion;
  std::deque<std::shared_ptr<adu::common::sensor::PointCloud>> fusion_deque_;
  uint64_t index_ = 0;
  uint64_t seq_ = 0;
  uint64_t size_ = 8;
};

CLASS_LOADER_REGISTER_COMPONENT(Compeight_left);

class Compeight_right : public Component<adu::common::sensor::PointCloud,
    adu::common::sensor::INSPVA> {
 public:
  int Init() override {
     cybertron::proto::VelodyneConfig velodyne_config;
    if (ParseConfig(&velodyne_config) != cybertron::SUCC) {
      LOG_ERROR << DRIVER_ERROR << LIDAR_CONF_PARSE_ERR << " Parse config file error";
      return cybertron::FAIL;
    }

    // _fusion.reset(new Calibrate(velodyne_config));

    fusion_deque_.resize(size_);
    for (int i = 0; i < size_; ++i) {
      fusion_deque_[i] = std::make_shared<adu::common::sensor::PointCloud>();
      if (fusion_deque_[i] == nullptr) {
        LOG_ERROR << DRIVER_ERROR << LIDAR_INIT_ERR << " fail to make shared";
        return FAIL;
      }
      
      fusion_deque_[i]->mutable_point()->Reserve(140000);
    }
     cout<<"Compeight_right**************init successfully"<<endl;
    return SUCC;
  }

  int Proc(
      const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_first,
      const std::shared_ptr<adu::common::sensor::INSPVA const>& INSPVA) override{
    
    if (index_ >= size_) {
      index_ = 0;
    }
    //  cout<<"prockkkkkkkkkkkkk"<<endl;
    auto point_cloud = fusion_deque_.at(index_++);
    point_cloud->Clear();
    if (_fusion->calib_eight_right(point_cloud_first, INSPVA, point_cloud)) {
      seq_++;
      point_cloud->mutable_header()->set_sequence_num(seq_);
      Send(point_cloud);
      return SUCC;
    } else {
      return FAIL;
    }
 
 }

 private:
  std::unique_ptr<Calibrate_eight> _fusion;
  std::deque<std::shared_ptr<adu::common::sensor::PointCloud>> fusion_deque_;
  uint64_t index_ = 0;
  uint64_t seq_ = 0;
  uint64_t size_ = 8;
};

CLASS_LOADER_REGISTER_COMPONENT(Compeight_right);

class Compvelodyne_add : public Component<adu::common::sensor::PointCloud,adu::common::sensor::PointCloud,
adu::common::sensor::PointCloud> {
   
 public:
  int Init() override {
     cybertron::proto::VelodyneConfig velodyne_config;
    // if (ParseConfig(&velodyne_config) != cybertron::SUCC) {
    //   LOG_ERROR << DRIVER_ERROR << LIDAR_CONF_PARSE_ERR << " Parse config file error";
    //   return cybertron::FAIL;
    // }

    // _fusion.reset(new Calibrate(velodyne_config));

    fusion_deque_.resize(size_);
    for (int i = 0; i < size_; ++i) {
      fusion_deque_[i] = std::make_shared<adu::common::sensor::PointCloud>();
      if (fusion_deque_[i] == nullptr) {
        LOG_ERROR << DRIVER_ERROR << LIDAR_INIT_ERR << " fail to make shared";
        return FAIL;
      }
      
      fusion_deque_[i]->mutable_point()->Reserve(140000);
    }
    cout<<"Compvelodyne_add*************"<<endl;
    return SUCC;
  }

   int Proc(
      const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_left,
      const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_right,
      const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_back
      ) override{ 
    if (index_ >= size_) {
      index_ = 0;
    }
    auto point_cloud = fusion_deque_.at(index_++);
    point_cloud->Clear();
    // if (_fusion->cut(point_cloud_first, gps_posit,heading, point_cloud)) {
     if (_fusion->velodyne_add(point_cloud_left,point_cloud_right,point_cloud_back,point_cloud)) {
      seq_++;
      point_cloud->mutable_header()->set_sequence_num(seq_);
      Send(point_cloud);
      return SUCC;
    } else {
      return FAIL;
    }
 
 }

 private:
  std::unique_ptr<Calibrate_eight> _fusion;
  std::deque<std::shared_ptr<adu::common::sensor::PointCloud>> fusion_deque_;
  uint64_t index_ = 0;
  uint64_t seq_ = 0;
  uint64_t size_ = 8;
};

CLASS_LOADER_REGISTER_COMPONENT(Compvelodyne_add);

class Compvelodyne_fusion : public Component<adu::common::sensor::PointCloud,adu::common::sensor::PointCloud> {
   
 public:
  int Init() override {
     cybertron::proto::VelodyneConfig velodyne_config;
    if (ParseConfig(&velodyne_config) != cybertron::SUCC) {
      LOG_ERROR << DRIVER_ERROR << LIDAR_CONF_PARSE_ERR << " Parse config file error";
      return cybertron::FAIL;
    }

    // _fusion.reset(new Calibrate(velodyne_config));

    fusion_deque_.resize(size_);
    for (int i = 0; i < size_; ++i) {
      fusion_deque_[i] = std::make_shared<adu::common::sensor::PointCloud>();
      if (fusion_deque_[i] == nullptr) {
        LOG_ERROR << DRIVER_ERROR << LIDAR_INIT_ERR << " fail to make shared";
        return FAIL;
      }
      
      fusion_deque_[i]->mutable_point()->Reserve(140000);
    }
    cout<<"Compvelodyne_fusion*************"<<endl;
    return SUCC;
  }

   int Proc(
      const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_left,
      const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_right
      ) override{ 
    if (index_ >= size_) {
      index_ = 0;
    }
    auto point_cloud = fusion_deque_.at(index_++);
    point_cloud->Clear();
    // if (_fusion->cut(point_cloud_first, gps_posit,heading, point_cloud)) {
     if (_fusion->velodyne_fusion(point_cloud_left,point_cloud_right,point_cloud)) {
      seq_++;
      point_cloud->mutable_header()->set_sequence_num(seq_);
      Send(point_cloud);
      return SUCC;
    } else {
      return FAIL;
    }
 
 }

 private:
  std::unique_ptr<Calibrate_eight> _fusion;
  std::deque<std::shared_ptr<adu::common::sensor::PointCloud>> fusion_deque_;
  uint64_t index_ = 0;
  uint64_t seq_ = 0;
  uint64_t size_ = 8;
};

CLASS_LOADER_REGISTER_COMPONENT(Compvelodyne_fusion);

}
}
}  // namespace cybertron

#endif  // ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_COMPONENT_COMPONENT_COMPENSATOR_H
