
#ifndef CYBERTRON_ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_LIB_PCAP_INPUT_H
#define CYBERTRON_ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_LIB_PCAP_INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>

#include "input.h"

namespace cybertron {
namespace drivers {
namespace velodyne {

class PcapInput : public Input {
 public:
  PcapInput(double packet_rate, const std::string& filename,
            bool read_once = false, bool read_fast = false,
            double repeat_delay = 0.0);
  virtual ~PcapInput();

  void init();
  int get_firing_data_packet(adu::common::sensor::VelodynePacket* pkt);
  int get_positioning_data_packtet(const NMEATimePtr& nmea_time);

 private:
  std::string _filename;
  FILE* _fp;
  pcap* _pcap;
  char _errbuf[PCAP_ERRBUF_SIZE];
  bool _empty;
  bool _read_once;
  bool _read_fast;
  double _repeat_delay;
  double _packet_rate;
};
}
}
}  // velodyne_driver namespace

#endif  // CYBERTRON_ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_LIB_PCAP_INPUT_H
