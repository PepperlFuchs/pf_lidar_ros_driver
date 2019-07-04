#include "pf_driver/transport.hpp"
#include "pf_driver/r2000/data_type_r2000.hpp"

class TransportR2000 : public Transport<PacketHeaderR2000> {

private:
    std::size_t get_data_size(std::uint16_t type) {
        switch(type) {
            case 65 : return sizeof(PacketTypeA::data);
            case 66 : return sizeof(PacketTypeB::data);
            case 67 : return sizeof(PacketTypeC::data);
        }
        return sizeof(PacketTypeC::data);
    }

    void fill_scan_data(ScanData& scandata, PacketHeaderR2000* p_header, std::string str) override {
        std::uint32_t *data = reinterpret_cast<std::uint32_t*>((char*)str.c_str());
        PacketTypeC *p_c = new PacketTypeC;
        p_c->header = *p_header;
        p_c->data.distance =   (*data & 0x000FFFFF);
        p_c->data.amplitude = ((*data & 0xFFF00000) >> 20);

        scandata.distance_data.push_back(p_c->data.distance);
        scandata.amplitude_data.push_back(p_c->data.amplitude);
    }

    std::string get_packet_type() override {
        return "C";
    }
};