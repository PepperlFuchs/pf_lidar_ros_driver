#include "pf_driver/pfsdp_protocol.hpp"
#include "pf_driver/communication.hpp"

class PFDriver {
public:
    bool connect(const std::string hostname, int port, int major_version) {
        protocol_interface = new PFSDPBase(hostname);
        auto opi = protocol_interface->get_protocol_info();
        if(opi.version_major != major_version) {
            std::cerr << "ERROR: Could not connect to laser range finder!" << std::endl;
            return false;
        }
        if(opi.version_major != major_version) {
            std::cerr << "ERROR: Wrong protocol version (version_major=" << opi.version_major << ", version_minor=" << opi.version_minor << ")" << std::endl;
            return false;
        }
        protocol_info = opi;
        parameters = protocol_interface->get_parameter(protocol_interface->list_parameters());
        is_connected = true;
        watchdog_feed_time = 0;
        return true;
    }

    ~PFDriver() {
        disconnect();
    }

    bool startCapturingTCP() {
        if( !checkConnection() )
            return false;

        handle_info = protocol_interface->request_handle_tcp('C', 0);
        
        data_receiver = new TCPConnection(handle_info.hostname, handle_info.port);
        try {
            data_receiver->connect();
        } catch (std::exception& e) {
            std::cerr << "Exception: " <<  e.what() << std::endl;
            return false;
        }
        if(!data_receiver->is_connected() || !protocol_interface->start_scanoutput(handle_info.handle))
            return false;
        start_read();
        std::cout << "started receiving data" << std::endl;
        feed_timeout = std::floor(std::max((handle_info.watchdog_timeout / 1000.0 / 3.0), 1.0));
        is_capturing = true;
        return true;
    }

    bool startCapturingUDP() {
        if( !checkConnection() )
            return false;

        data_receiver = new UDPConnection();
        try {
            data_receiver->connect();
        } catch (std::exception& e) {
            std::cerr << "Exception: " <<  e.what() << std::endl;
            return false;
        }
        if(!data_receiver->is_connected() )
            return false;
        std::string udp_port = data_receiver->get_port();
        std::cout << "port: " << udp_port << std::endl;

        handle_info = protocol_interface->request_handle_udp(udp_port, 'C', 0);
        if(!protocol_interface->start_scanoutput(handle_info.handle) )
            return false;

        start_read();
        feed_timeout = std::floor(std::max((handle_info.watchdog_timeout/1000.0/3.0),1.0));
        is_capturing = true;
        return true;
    }

    bool stopCapturing() {
        if( !is_capturing || !protocol_interface )
            return false;

        bool return_val = checkConnection();

        return_val = return_val && protocol_interface->stop_scanoutput(handle_info.handle);

        delete data_receiver;
        data_receiver = 0;

        is_capturing = false;
        return_val = return_val && protocol_interface->release_handle(handle_info.handle);
        handle_info = HandleInfo();
        return return_val;
    }

    bool isConnected() {
        return is_connected; 
    }

    bool checkConnection() {
        // if( !protocol_interface || !isConnected() || !protocol_interface->get_protocol_info() ) {
        //     std::cerr << "ERROR: No connection to laser range finder or connection lost!" << std::endl;
        //     return false;
        // }
        return true;
    }

    const std::map< std::string, std::string >& getParametersCached() const {return parameters;}

    void start_read() {
        if(data_receiver)
            data_receiver->start_read();
    }

    ScanData getScan() {
        this->feedWatchdog();
        if( data_receiver )
            return data_receiver->get_scan();
        else {
            std::cerr << "ERROR: No scan capturing started!" << std::endl;
            return ScanData();
        }
    }

    ScanData getFullScan() {
        this->feedWatchdog();
        if(data_receiver)
            return data_receiver->get_full_scan();
        else {
            std::cerr << "ERROR: No scan capturing started!" << std::endl;
            return ScanData();
        }
    }

    std::size_t getScansAvailable() const {
        if( data_receiver )
            return data_receiver->get_scans_available();
        else {
            std::cerr << "ERROR: No scan capturing started!" << std::endl;
            return 0;
        }
    }

    std::size_t getFullScansAvailable() const {
        if( data_receiver )
            return data_receiver->get_full_scans_available();
        else {
            std::cerr << "ERROR: No scan capturing started!" << std::endl;
            return 0;
        }
    }

    void disconnect() {
        if(isCapturing())
            stopCapturing();

        delete data_receiver;
        delete protocol_interface;
        data_receiver = 0;
        protocol_interface = 0;

        is_capturing = false;
        is_connected = false;

        handle_info = HandleInfo();
        protocol_info = ProtocolInfo();
        parameters = std::map< std::string, std::string >();
    }

    bool isCapturing() {
        return is_capturing && data_receiver->is_connected();
    }

    const std::map< std::string, std::string >& getParameters() {
        if(protocol_interface)
            parameters = protocol_interface->get_parameter(protocol_interface->list_parameters());
        return parameters;
    }

    bool setScanFrequency(unsigned int frequency) {
        if( !protocol_interface )
            return false;
        return protocol_interface->set_parameter({{"scan_frequency", std::to_string(frequency)}});
    }

    bool setSamplesPerScan(unsigned int samples) {
        if( !protocol_interface )
            return false;
        return protocol_interface->set_parameter({{"samples_per_scan", std::to_string(samples)}});
    }

    bool rebootDevice() {
        if( !protocol_interface )
            return false;
        return protocol_interface->reboot_device();
    }

    bool resetParameters(const std::vector<std::string> &names) {
        if( !protocol_interface )
            return false;
        return protocol_interface->reset_parameter(names);
    }

    bool setParameter(const std::string &name, const std::string &value) {
        if(!protocol_interface)
            return false;
        return protocol_interface->set_parameter({{name, value}});
    }

    void feedWatchdog(bool feed_always = false) {
        const double current_time = std::time(0);

        if(feed_always || watchdog_feed_time < (current_time - feed_timeout)) {
            if(!protocol_interface->feed_watchdog(handle_info.handle))
                std::cerr << "ERROR: Feeding watchdog failed!" << std::endl;
            watchdog_feed_time = current_time;
        }
    }

private:
    PFSDPBase *protocol_interface;
    ProtocolInfo protocol_info;
    std::map< std::string, std::string > parameters;
    HandleInfo handle_info;
    Connection *data_receiver;

    double watchdog_feed_time;
    double feed_timeout;

    bool is_connected;
    bool is_capturing;
};