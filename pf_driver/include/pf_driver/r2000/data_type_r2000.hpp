// Copyright 2019 Fraunhofer IPA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PF_DRIVER_R2000_DATATYPE_H
#define PF_DRIVER_R2000_DATATYPE_H

#include "pf_driver/data_type.h"

#pragma pack(1)

struct PacketHeaderR2000
{
    std::uint16_t magic;             //magic byte (0xa25c) marking the beginning of a packet
    std::uint16_t packet_type;       //scan data packet type (0x0041 - 'A'; 0x0042 - 'B'; 0x0043 - 'C')
    std::uint32_t packet_size;       //overall packet size in bytes (header + payload)
    std::uint16_t header_size;       //header size in bytes (offset to payload data)
    std::uint16_t scan_number;       //sequence number for scan (incremented for every scan, starting with 0, overflows)
    std::uint16_t packet_number;     //sequence number for packet (counting packets of a particular scan, starting with 1)
    std::uint64_t timestamp_raw;     //raw timestamp of first scan point in this packet in NTP time format
    std::uint64_t timestamp_sync;    //synchronized timestamp of first scan point in this packet in NTP time format
                                     //(currenty not available and and set to zero)
    std::uint32_t status_flags;      //scan status flags (TODO: should this be a struct? or provide masking to extract specific flags)
    std::uint32_t scan_frequency;    //frequency of head rotation  (1/1000Hz)
    std::uint16_t num_points_scan;   //number of scan points (samples) within complete scan
    std::uint16_t num_points_packet; //total number of scan points within this packet
    std::uint16_t first_index;       //index of first scan point within this packet
    std::int32_t first_angle;        //absolute angle of first scan point within this packet  (1/10000°)
    std::int32_t angular_increment;  //delta between two succeding scan points (1/10000°); CCW rotation: +ve, CW rotation: -ve
    std::uint32_t iq_input;          //reserved - all bits zero for devices without switching I/Q
    std::uint32_t iq_overload;       //reserved - all bits zero for devices without switching I/Q
    //std::uint8_t padding[3];          //0-3 bytes padding to align header size to 32bit boundary (TODO: should I use vector instead?)
};

#pragma pack()

#endif