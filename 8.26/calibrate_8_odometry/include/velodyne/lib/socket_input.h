
#ifndef CYBERTRON_ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_LIB_SOCKET_INPUT_H
#define CYBERTRON_ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_LIB_SOCKET_INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>

#include "input.h"

namespace cybertron {
namespace drivers {
namespace velodyne {

static const int POLL_TIMEOUT = 1000;  // one second (in msec)

/** @brief Live Velodyne input from socket. */
class SocketInput : public Input {
 public:
  SocketInput();
  virtual ~SocketInput();
  void init(uint32_t port);
  int get_firing_data_packet(adu::common::sensor::VelodynePacket* pkt);
  int get_positioning_data_packtet(const NMEATimePtr& nmea_time);

 private:
  uint32_t _sockfd;
  int _port;
  bool input_available(int timeout);
};
}
}
}  // velodyne_driver namespace

#endif  // CYBERTRON_ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_LIB_SOCKET_INPUT_H
