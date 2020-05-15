#pragma once

#include <vector>
#include "pf_driver/pf/reader.h"

class PFPacket
{
public:
    std::uint16_t magic;           // magic byte (0xa25c) marking the beginning of a packet
    std::uint16_t packet_type;     // scan data packet type (0x0041 - 'A'; 0x0042 - 'B'; 0x0043 - 'C')
    std::uint32_t packet_size;     // overall packet size in bytes (header + payload)
    std::uint16_t header_size;     // header size in bytes (offset to payload data)
    std::uint16_t scan_number;     // sequence number for scan (incremented for every scan, starting with 0, overflows)
    std::uint16_t packet_number;   // sequence number for packet (counting packets of a particular scan, starting with 1)
};

class PFR2000Packet : public PFPacket
{
public:
    std::uint64_t timestamp_raw;      // raw timestamp of first scan point in this packet in NTP time format
    std::uint64_t timestamp_sync;     // synchronized timestamp of first scan point in this packet in NTP time format
                                      //(currenty not available and and set to zero)
    std::uint32_t status_flags;       // scan status flags (TODO: should this be a struct? or provide masking to extract
                                      // specific flags)
    std::uint32_t scan_frequency;     // frequency of head rotation  (1/1000Hz)
    std::uint16_t num_points_scan;    // number of scan points (samples) within complete scan
    std::uint16_t num_points_packet;  // total number of scan points within this packet
    std::uint16_t first_index;        // index of first scan point within this packet
    std::int32_t first_angle;         // absolute angle of first scan point within this packet  (1/10000°)
    std::int32_t angular_increment;   // delta between two succeding scan points (1/10000°); CCW rotation: +ve, CW
                                      // rotation: -ve
    std::uint32_t iq_input;           // reserved - all bits zero for devices without switching I/Q
    std::uint32_t iq_overload;        // reserved - all bits zero for devices without switching I/Q
    std::uint64_t iq_timestamp_raw;   // raw timestamp for status of switching I/Q
    std::uint64_t iq_timestamp_sync;  // synchronized timestamp for status of switching I/Q
    std::vector<std::uint8_t> padding; // 0-3 bytes padding to align header size to 32bit boundary (TODO: should I use
};

class PFR2000Packet_A : public PFR2000Packet
{
public:
    std::vector<std::uint32_t> distance;        // measured distance (mm)
};

class PFR2000Packet_B : public PFR2000Packet
{
public:
    std::vector<std::uint32_t> distance;        // measured distance (mm)
    std::vector<std::uint16_t> amplitude;       // measured amplitude (mm)
    std::vector<std::uint8_t> payload_padding;  // 0 or 2 bytes padding
};

class PFR2000Packet_C : public PFR2000Packet
{
public:
    std::vector<std::uint32_t> dist_amp;        //TODO: need to check this
};

class PFR2300Packet_C1 : public PFPacket
{
public:
    std::uint16_t layer_index;    // vertical layer index (0..3)
    std::int32_t layer_inclination;   // vertical layer inclination [1/10000 degree]
    std::uint64_t timestamp_raw;      // raw timestamp of first scan point in this packet in NTP time format
    std::uint64_t reserved1;          // reserved - all bits zero for devices without switching I/Q
    std::uint32_t status_flags;       // scan status flags (TODO: should this be a struct? or provide masking to extract
                                        // specific flags)
    std::uint32_t scan_frequency;     // frequency of head rotation  (1/1000Hz)
    std::uint16_t num_points_scan;    // number of scan points (samples) within complete scan
    std::uint16_t num_points_packet;  // total number of scan points within this packet
    std::uint16_t first_index;        // index of first scan point within this packet
    std::int32_t first_angle;         // absolute angle of first scan point within this packet  (1/10000°)
    std::int32_t angular_increment;   // delta between two succeding scan points (1/10000°); CCW rotation: +ve, CW
                                        // rotation: -ve
    std::uint32_t reserved2;          // reserved - all bits zero for devices without switching I/Q
    std::uint32_t reserved3;          // reserved - all bits zero for devices without switching I/Q
    std::uint64_t reserved4;          // reserved - all bits zero for devices without switching I/Q
    std::uint64_t reserved5;          // reserved - all bits zero for devices without switching I/Q
    std::vector<uint32_t> padding;    // multiples of 4 bytes

    std::vector<uint32_t> dist_amp;
};