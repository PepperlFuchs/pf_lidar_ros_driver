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

#include "pf_driver/transport.hpp"
#include "pf_driver/r2300/data_type_r2300.hpp"

class TransportR2300 : public Transport<PacketHeaderR2300>
{
public:
    std::pair<bool, size_t> parse_header(std::string str) override
    {
        int start = find_packet_start(get_packet_type(), str);
        int len = str.length() - start;
        str.erase(str.begin(), str.end() - len);
        if (start >= 0 && str.size() >= get_header_size())
        {
            p_header = reinterpret_cast<PacketHeaderR2300 *>((char *)str.c_str());

            std::uint16_t num_points = p_header->num_points_packet;
            int packet_num = p_header->packet_number;

            str.erase(str.begin(), str.end() - (str.size() - p_header->header_size));
            std::unique_lock<std::mutex> lock(data_mutex);

            if (packet_num == 1 || scans.empty())
            {
                scans.emplace_back();
                data_notifier.notify_one();
            }
            ScanData &scandata = scans.back();

            for (int i = 0; i < num_points; i++)
            {
                std::string d = str.substr(0, sizeof(std::uint32_t));
                fill_scan_data(scandata, p_header, d);
                str.erase(str.begin(), str.end() - (str.size() - sizeof(std::uint32_t)));
            }
        }
        return std::pair<bool, size_t>(false, 100000);
    }

protected:
    std::size_t get_data_size(std::uint16_t type)
    {
        return sizeof(PacketTypeC1::data);
    }

    void fill_scan_data(ScanData &scandata, PacketHeaderR2300 *p_header, std::string str) override
    {
        std::uint32_t *data = reinterpret_cast<std::uint32_t *>((char *)str.c_str());
        PacketTypeC1 *p_c = new PacketTypeC1;
        p_c->header = *p_header;
        p_c->data.distance = (*data & 0x000FFFFF);
        p_c->data.amplitude = ((*data & 0xFFF00000) >> 20);

        scandata.distance_data.push_back(p_c->data.distance);
        scandata.amplitude_data.push_back(p_c->data.amplitude);
    }

    std::string get_packet_type() override
    {
        return "C1";
    }
};