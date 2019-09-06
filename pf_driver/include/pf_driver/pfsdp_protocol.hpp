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

#include <boost/algorithm/string.hpp>
#include "pf_driver/http_helpers.hpp"

struct ProtocolInfo
{
    std::string protocol_name;         //protocol name, defaults to "pfsdp"
    int version_major;                 //major version of protocol
    int version_minor;                 //minor version of protocol
    std::vector<std::string> commands; //list of available commands
};

struct HandleInfo
{
    static const int HANDLE_TYPE_TCP = 0;
    static const int HANDLE_TYPE_UDP = 1;

    int handle_type;
    std::string hostname;
    std::string port;
    std::string handle;
    char packet_type;
    int start_angle;
    bool watchdog_enabled;
    int watchdog_timeout;
};

class KV : public std::pair<std::string, std::string>
{
    template <typename T>
    static std::string make_list(bool first, const T &t)
    {
        std::stringstream s;
        if (!first)
            s << ";";
        s << t;
        return s.str();
    }

    template <typename T, typename... Ts>
    static std::string make_list(bool first, const T &t, Ts &&... list)
    {
        std::stringstream s;
        s << make_list(first, t);
        s << make_list(false, list...);
        return s.str();
    }

    static std::string make_list(bool first, std::vector<std::string> list)
    {
        std::stringstream s;
        std::copy(list.begin(), list.end() - 1, std::ostream_iterator<std::string>(s, ";"));
        s << list.back();

        return s.str();
    }

public:
    template <typename... Ts>
    KV(const std::string &k, const Ts &... list) : std::pair<std::string, std::string>(k, make_list(true, list...)) {}
};

const std::vector<std::string> split(std::string str, const char delim = ';')
{
    std::vector<std::string> results;
    boost::split(results, str, [](char c) { return c == ';'; });
    return results;
}

class PFSDPBase
{
private:
    using HTTPInterfacePtr = std::unique_ptr<HTTPInterface>;
    HTTPInterfacePtr http_interface;
    std::string hostname;

    const std::map<std::string, std::string> get_request(const std::string command, std::vector<std::string> json_keys = std::vector<std::string>(),
                                                         std::initializer_list<param_type> query = std::initializer_list<param_type>())
    {
        std::vector<std::string> keys = {"error_code", "error_text"};
        keys.insert(keys.end(), json_keys.begin(), json_keys.end());
        std::map<std::string, std::string> json_resp = http_interface->get(keys, command, query);

        return json_resp;
    }

public:
    PFSDPBase(const utility::string_t &host) : hostname(host),
                                               http_interface(new HTTPInterface(host, "cmd")) {}

    const std::vector<std::string> list_parameters()
    {
        auto resp = get_request("list_parameters", {"parameters"});
        return split(resp["parameters"]);
    }

    bool reboot_device()
    {
        get_request("reboot_device");
        return true;
    }

    void factory_reset()
    {
        get_request("factory_reset");
    }

    bool release_handle(std::string handle)
    {
        get_request("release_handle", {""}, {KV("handle", handle)});
        return true;
    }

    ProtocolInfo get_protocol_info()
    {
        auto resp = get_request("get_protocol_info", {"protocol_name", "version_major", "version_minor", "commands"});
        ProtocolInfo opi;
        opi.version_major = atoi(resp["version_major"].c_str());
        opi.version_minor = atoi(resp["version_minor"].c_str());
        opi.protocol_name = resp["protocol_name"];
        return opi;
    }

    template <typename... Ts>
    bool set_parameter(const std::initializer_list<param_type> params)
    {
        get_request("set_parameter", {""}, params);
        return true;
    }

    template <typename... Ts>
    std::map<std::string, std::string> get_parameter(const Ts &... ts)
    {
        return get_request("get_parameter", ts..., {KV("list", ts...)});
    }

    template <typename... Ts>
    bool reset_parameter(const Ts &... ts)
    {
        get_request("reset_parameter", {""}, {KV("list", ts...)});
        return true;
    }

    HandleInfo request_handle_tcp(const char packet_type, const int start_angle)
    {
        auto resp = get_request("request_handle_tcp", {"handle", "port"}, {KV("packet_type", std::string(1, packet_type)), KV("start_angle", start_angle)});
        HandleInfo handle_info;
        handle_info.hostname = hostname;
        handle_info.handle = resp["handle"];
        handle_info.port = resp["port"];
        handle_info.packet_type = packet_type;
        handle_info.start_angle = start_angle;
        handle_info.watchdog_enabled = true;
        handle_info.watchdog_timeout = 60000;
        return handle_info;
    }

    HandleInfo request_handle_udp(const std::string port, const char packet_type, const int start_angle)
    {
        auto resp = get_request("request_handle_udp", {"handle", "port"}, {KV("address", "10.0.10.1"), KV("port", port)});
        HandleInfo handle_info;
        handle_info.handle = resp["handle"];
        handle_info.port = resp["port"];
        handle_info.packet_type = packet_type;
        handle_info.start_angle = start_angle;
        handle_info.watchdog_enabled = true;
        handle_info.watchdog_timeout = 60000;
        return handle_info;
    }

    bool start_scanoutput(std::string handle)
    {
        get_request("start_scanoutput", {""}, {{"handle", handle}});
        return true;
    }

    bool stop_scanoutput(std::string handle)
    {
        get_request("stop_scanoutput", {""}, {{"handle", handle}});
        return true;
    }

    bool feed_watchdog(std::string handle)
    {
        get_request("feed_watchdog", {""}, {{"handle", handle}});
        return true;
    }
};
