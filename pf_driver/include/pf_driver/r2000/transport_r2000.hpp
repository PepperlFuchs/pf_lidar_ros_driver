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
#include "pf_driver/r2000/data_type_r2000.hpp"

class TransportR2000 : public Transport<PacketHeaderR2000>
{

private:
    std::size_t get_data_size(std::uint16_t type)
    {
        switch (type)
        {
        case 65:
            return sizeof(PacketTypeA::data);
        case 66:
            return sizeof(PacketTypeB::data);
        case 67:
            return sizeof(PacketTypeC::data);
        }
        return sizeof(PacketTypeC::data);
    }

    void fill_scan_data(ScanData &scandata, PacketHeaderR2000 *p_header, std::string str) override
    {
        std::uint32_t *data = reinterpret_cast<std::uint32_t *>((char *)str.c_str());
        PacketTypeC *p_c = new PacketTypeC;
        p_c->header = *p_header;
        p_c->data.distance = (*data & 0x000FFFFF);
        p_c->data.amplitude = ((*data & 0xFFF00000) >> 20);

        scandata.distance_data.push_back(p_c->data.distance);
        scandata.amplitude_data.push_back(p_c->data.amplitude);
    }

    std::string get_packet_type() override
    {
        return "C";
    }
};