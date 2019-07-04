#include <iostream>
#include <vector>
#include <string>
#include <deque>
#include <mutex>
#include <condition_variable>

struct ScanData {
    std::vector<std::uint32_t> distance_data;   
    std::vector<std::uint32_t> amplitude_data;  
};

template<typename PH>
class Transport {
public:
    Transport() {
        scan_angle = 0;
    }
    ~Transport() {
    }

    virtual std::pair<bool, size_t> parse_header(std::string str) {
        header_str.append(str);
        int start = find_packet_start(get_packet_type(), header_str);
        int len = header_str.length() - start;
        header_str.erase(header_str.begin(), header_str.end() - len);
        std::uint32_t status_flag = 0;
        if(start >= 0 && header_str.size() >= get_header_size()) {
            p_header = reinterpret_cast<PH*>((char*)header_str.c_str());
            total_data_size =  p_header->packet_size - get_header_size();
            header_str.clear();
            return std::pair<bool, size_t>(true, total_data_size);
        }
        return std::pair<bool, size_t>(false, get_header_size() - str.size());
    }

    std::pair<bool, size_t> parse_data(std::string str) {
        data_str.append(str);
        if(data_str.size() >= total_data_size) {
            get_data(data_str, p_header->num_points_packet);
            data_str.clear();
            total_data_size = 0;
            return std::pair<bool, size_t>(true, get_header_size());
        }
        return std::pair<bool, size_t>(false, total_data_size - data_str.size());
    }

    ScanData get_scan() {
        std::unique_lock<std::mutex> lock(data_mutex);
        ScanData data(std::move(scans.front()));
        scans.pop_front();
        return data;
    }

    ScanData get_full_scan() {
        std::unique_lock<std::mutex> lock(data_mutex);
        while(scans.size() < 2) {
            data_notifier.wait_for(lock, std::chrono::seconds(1));
        }
        ScanData data;
        if(scans.size() >= 2) {
            data = ScanData(std::move(scans.front()));
            scans.pop_front();
        }
        return data;
    }

    std::size_t get_full_scans_available() const {
        if(scans.size() == 0)
            return 0;
        else
            return scans.size()-1;
    }

    std::size_t get_scans_available() const {
        return scans.size();
    }

    std::size_t get_header_size() {
        return sizeof(PH);
    }

protected:
    int count = 0;

    virtual std::size_t get_data_size(std::uint16_t type) = 0;
    virtual void fill_scan_data(ScanData& scandata, PH* p_header, std::string str) = 0;
    virtual std::string get_packet_type() = 0;

    void get_data(std::string str, std::uint16_t num_points) {
        str.erase(str.begin(), str.end() - (str.size() - (p_header->header_size - get_header_size())));
        std::unique_lock<std::mutex> lock(data_mutex);

        if(p_header->packet_number == 1 || scans.empty()) {
            scans.emplace_back();
            data_notifier.notify_one();
        }
        ScanData& scandata = scans.back();

        for(int i = 0; i < num_points; i++) {
            std::string d = str.substr(0, sizeof(std::uint32_t));
            fill_scan_data(scandata, p_header, d);
            str.erase(str.begin(), str.end() - (str.size() - sizeof(std::uint32_t)));
        }
    }

    int find_packet_start(std::string type, std::string str) {
        for(int i = 0; i < str.size() - 4; i++) {
            if(((unsigned char) str[i]) == 0x5c &&
               ((unsigned char) str[i+1]) == 0xa2 &&
               ((unsigned char) str[i+2]) == type[0] &&
               ((unsigned char) str[i+3]) == type[1]) {
                    return i;
            }
        }
        return -1;
    }

    double scan_angle;

    PH* p_header;
    std::string header_str, data_str;
    std::deque<ScanData> scans;

    std::condition_variable data_notifier;
    std::mutex data_mutex;
    std::size_t total_data_size;
};
