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

#ifndef ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_PARSER_VELODYNE_PARSER_H
#define ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_PARSER_VELODYNE_PARSER_H

#include <errno.h>
#include <map>
#include <set>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>

#include "cybertron/time/time.h"
#include "proto/sensor_velodyne.pb.h"
#include "proto/sensor_pointcloud.pb.h"
#include "proto/lidars_filter_config.pb.h"  //tofind
 #include "proto/velodyne_config.pb.h"

#include "velodyne/lib/calibration.h"
#include "velodyne/lib/const_variables.h"
#include "velodyne/lib/data_type.h"

namespace cybertron {
namespace drivers {
namespace velodyne {

/** \brief Velodyne data conversion class */
class VelodyneParser {
 public:
  VelodyneParser() {}
  VelodyneParser(cybertron::proto::VelodyneConfig& config);
  virtual ~VelodyneParser() {}

  /** \brief Set up for data processing.
   *
   *  Perform initializations needed before data processing can
   *  begin:
   *
   *    - read device-specific angles calibration
   *
   *  @param private_nh private node handle for ROS parameters
   *  @returns 0 if successful;
   *           errno value for failure
   */
  virtual void generate_pointcloud(
      const std::shared_ptr<adu::common::sensor::VelodyneScan const>& scan_msg,
      std::shared_ptr<adu::common::sensor::PointCloud>& out_msg) = 0;
  virtual void setup();
  virtual uint32_t GetPointSize() = 0;
  // order point cloud fod IDL by velodyne model
  virtual void order(std::shared_ptr<adu::common::sensor::PointCloud>& cloud) = 0;

  const Calibration& get_calibration() { return _calibration; }
  double get_last_timestamp() { return _last_time_stamp; }

 protected:
  const float (*_inner_time)[12][32];
  /**
   * \brief Calibration file
   */
  Calibration _calibration;
  float _sin_rot_table[ROTATION_MAX_UNITS];
  float _cos_rot_table[ROTATION_MAX_UNITS];
  cybertron::proto::VelodyneConfig _config;
  std::set<std::string> _filter_set;
  int _filter_grading;
  // Last Velodyne packet time stamp. (Full time)
  double _last_time_stamp;
  bool _need_two_pt_correction;
  uint32_t point_index_ = 0;
  Mode _mode;

  adu::common::sensor::PointXYZIT get_nan_point(uint64_t timestamp);
  void init_angle_params(double view_direction, double view_width);
  /**
   * \brief Compute coords with the data in block
   *
   * @param tmp A two bytes union store the value of laser distance infomation
   * @param index The index of block
   */
  void compute_coords(const union RawDistance& raw_distance,
                      const LaserCorrection& corrections,
                      const uint16_t& rotation,
                      adu::common::sensor::PointXYZIT* point);
  void compute_coords_beike(const float& raw_distance,
                     const uint16_t rotation,  const int laserChannel, adu::common::sensor::PointXYZIT* point); //beike

  bool is_scan_valid(int rotation, float distance);

  /**
   * \brief Unpack velodyne packet
   *
   */
  virtual void unpack(const adu::common::sensor::VelodynePacket& pkt,
                      std::shared_ptr<adu::common::sensor::PointCloud>& pc) = 0;

  uint64_t get_gps_stamp(double current_stamp, double& previous_stamp,
                         uint64_t& gps_base_usec);

  virtual uint64_t get_timestamp(double base_time, float time_offset,
                                 uint16_t laser_block_id) = 0;

  void timestamp_check(double timestamp);

  void init_sin_cos_rot_table(float* sin_rot_table, float* cos_rot_table,
                              uint16_t rotation, float rotation_resolution);

};  // class VelodyneParser

class Velodyne64Parser : public VelodyneParser {
 public:
  Velodyne64Parser(cybertron::proto::VelodyneConfig& config);
  ~Velodyne64Parser() {}

  void generate_pointcloud(
      const std::shared_ptr<adu::common::sensor::VelodyneScan const>& scan_msg,
      std::shared_ptr<adu::common::sensor::PointCloud>& out_msg);
  void order(std::shared_ptr<adu::common::sensor::PointCloud>& cloud);
  void setup();
  uint32_t GetPointSize() override;

 private:
  void set_base_time_from_packets(const adu::common::sensor::VelodynePacket& pkt);
  void check_gps_status(const adu::common::sensor::VelodynePacket& pkt);
  uint64_t get_timestamp(double base_time, float time_offset,
                         uint16_t laser_block_id);
  void unpack(const adu::common::sensor::VelodynePacket& pkt,
              std::shared_ptr<adu::common::sensor::PointCloud>& pc);
  void init_offsets();
  int intensity_compensate(const LaserCorrection& corrections,
                           const uint16_t& raw_distance, int intensity);
  // Previous Velodyne packet time stamp. (offset to the top hour)
  double _previous_packet_stamp[4];
  uint64_t _gps_base_usec[4];  // full time
  bool _is_s2;
  int _offsets[64];

};  // class Velodyne64Parser

class Velodyne32Parser : public VelodyneParser {
 public:
  Velodyne32Parser(cybertron::proto::VelodyneConfig& config);
  ~Velodyne32Parser() {}

  void generate_pointcloud(
      const std::shared_ptr<adu::common::sensor::VelodyneScan const>& scan_msg,
      std::shared_ptr<adu::common::sensor::PointCloud>& out_msg);
  void order(std::shared_ptr<adu::common::sensor::PointCloud>& cloud);
  uint32_t GetPointSize() override;

 private:
  uint64_t get_timestamp(double base_time, float time_offset,
                         uint16_t laser_block_id);
  void unpack(const adu::common::sensor::VelodynePacket& pkt,
              std::shared_ptr<adu::common::sensor::PointCloud>& pc);
  // Previous Velodyne packet time stamp. (offset to the top hour)
  double _previous_packet_stamp;
  uint64_t _gps_base_usec;  // full time

};  // class Velodyne32Parser

class Velodyne16Parser : public VelodyneParser {
 public:
  Velodyne16Parser(cybertron::proto::VelodyneConfig& config);
  ~Velodyne16Parser() {}

  void generate_pointcloud(
      const std::shared_ptr<adu::common::sensor::VelodyneScan const>& scan_msg,
      std::shared_ptr<adu::common::sensor::PointCloud>& out_msg);
  void order(std::shared_ptr<adu::common::sensor::PointCloud>& cloud);
  uint32_t GetPointSize() override;
  void setup() override;

 private:
  uint64_t get_timestamp(double base_time, float time_offset,
                         uint16_t laser_block_id);
  void unpack(const adu::common::sensor::VelodynePacket& pkt,
              std::shared_ptr<adu::common::sensor::PointCloud>& pc);
  // Previous Velodyne packet time stamp. (offset to the top hour)
  double _previous_packet_stamp;
  uint64_t _gps_base_usec;  // full time
  std::map<uint32_t, uint32_t> order_map_;
  uint32_t getOrderIndex(uint32_t index);
  void init_orderindex();

};  // class Velodyne32Parser

class VelodyneParserFactory {
 public:
  static VelodyneParser* create_parser(
      cybertron::proto::VelodyneConfig& config);
};

}  // namespace velodyne
}
}

#endif  // ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_PARSER_VELODYNE_PARSER_H
