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

#ifndef ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_DRIVER_DRIVER_H
#define ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_DRIVER_DRIVER_H

#include <string>

#include "proto/sensor_velodyne.pb.h"
#include "proto/velodyne_config.pb.h"
#include "cybertron/common/common.h"
#include "cybertron/dag_streaming/component.h"
#include "cybertron/time/time.h"

#include "velodyne/lib/socket_input.h"
#include "velodyne/lib/pcap_input.h"
#include "velodyne/lib/data_type.h"

namespace cybertron {
namespace drivers {
namespace velodyne {

class VelodyneDriver {
 public:
  VelodyneDriver();
  virtual ~VelodyneDriver() {}

  virtual bool poll(std::shared_ptr<adu::common::sensor::VelodyneScan>& scan) {
    return true;
  }
  virtual void init() = 0;

 protected:
  cybertron::proto::VelodyneConfig _config;
  std::shared_ptr<Input> _input;

  uint64_t _basetime;
  uint32_t _last_gps_time;
  int poll_standard(std::shared_ptr<adu::common::sensor::VelodyneScan>& scan);
  int poll_sync_count(std::shared_ptr<adu::common::sensor::VelodyneScan>& scan, bool main_frame);
  uint64_t _last_count;
  bool set_base_time();
  void set_base_time_from_nmea_time(const NMEATimePtr& nmea_time,
                                    uint64_t& basetime);
  void update_gps_top_hour(unsigned int current_time);
};

class Velodyne64Driver : public VelodyneDriver {
 public:
  Velodyne64Driver(cybertron::proto::VelodyneConfig& velodyne_config);
  ~Velodyne64Driver() {}

  void init();
  bool poll(std::shared_ptr<adu::common::sensor::VelodyneScan>& scan);
};

class Velodyne32Driver : public VelodyneDriver {
 public:
  Velodyne32Driver(cybertron::proto::VelodyneConfig& velodyne_config);
  ~Velodyne32Driver() {}

  void init();
  bool poll(std::shared_ptr<adu::common::sensor::VelodyneScan>& scan);
  void poll_positioning_packet();

 private:
  std::shared_ptr<Input> _positioning_input;
};

class Velodyne16Driver : public VelodyneDriver {
 public:
  Velodyne16Driver(cybertron::proto::VelodyneConfig& velodyne_config);
  ~Velodyne16Driver();

  void init();
  bool poll(std::shared_ptr<adu::common::sensor::VelodyneScan>& scan);
  void poll_positioning_packet();

 private:
  std::shared_ptr<Input> _positioning_input;
  std::thread positioning_thread_;
  std::atomic<bool> _running = {true};
};

class VelodyneDriverFactory {
 public:
  static VelodyneDriver* create_driver(
      cybertron::proto::VelodyneConfig& velodyne_config);
};
}
}
}  // namespace velodyne_driver

#endif  // ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_DRIVER_DRIVER_H
