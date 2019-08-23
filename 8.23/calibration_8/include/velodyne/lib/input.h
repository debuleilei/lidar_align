#ifndef CYBERTRON_ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_LIB_INPUT_H
#define CYBERTRON_ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_LIB_INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>

#include "proto/sensor_velodyne.pb.h"
#include "velodyne/lib/data_type.h"

namespace cybertron {
namespace drivers {
namespace velodyne {

static const size_t FIRING_DATA_PACKET_SIZE = 1206;
static const size_t POSITIONING_DATA_PACKET_SIZE = 256;//beike-256  velodyne-512;
static const size_t ETHERNET_HEADER_SIZE = 42;
static const int PCAP_FILE_END = -1;
static const int SOCKET_TIMEOUT = -2;
static const int RECIEVE_FAIL = -3;

/** @brief Pure virtual Velodyne input base class */
class Input {
 public:
  Input() {}
  virtual ~Input() {};

  /** @brief Read one Velodyne packet.
   *
   * @param pkt points to VelodynePacket message
   *
   * @returns 0 if successful,
   *          -1 if end of file
   *          > 0 if incomplete packet (is this possible?)
   */
  virtual int get_firing_data_packet(adu::common::sensor::VelodynePacket* pkt) = 0;
  virtual int get_positioning_data_packtet(const NMEATimePtr& nmea_time) = 0;
  virtual void init() {};
  virtual void init(uint32_t port) {
    (void)port;
  };

 protected:
  bool exract_nmea_time_from_packet(const NMEATimePtr& nmea_time,
                                    const uint8_t* bytes);
};
}
}
}  // velodyne_driver namespace

#endif  // CYBERTRON_ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_LIB_INPUT_H
