#include <iostream>
#include <string>
#include <memory>
#include <typeinfo>
#include <cpprest/http_client.h>

using namespace utility;                    // Common utilities like string conversions
using namespace web;                        // Common features like URIs.
using namespace web::http;                  // Common HTTP functionality
using namespace web::http::client;          // HTTP client features

using param_type = std::pair<std::string, std::string>;

std::string to_string(web::json::value val) {
    if(val.is_integer()) {
        return std::to_string(val.as_integer());
    } if(val.is_double()) {
        return std::to_string(val.as_double());
    }  if(val.is_string()) {
        return val.as_string();
    }
    if(val.is_array()) {
        auto arr =  val.as_array();
        std::string s = "";
        for(size_t i = 0; i < arr.size() - 1; i++) {
            s += to_string(arr[i]) + ";";
        }
        s += to_string(arr.at(arr.size() - 1));
        return s;
    }
    return std::string();
}

class Resource {
    void set_host(const utility::string_t& host) {
        builder.set_host(host);
    }

    void set_scheme(const utility::string_t& scheme) {
        builder.set_scheme(scheme);
    }

    const std::string uri;
    uri_builder builder;

public:
    Resource(const utility::string_t& host){
        set_scheme("http");
        set_host(host);
    }

    Resource append_query(const std::initializer_list<param_type> &list, bool do_encoding = false){
        for(const auto& p: list){
            builder.append_query(p.first, p.second, do_encoding);
        }
        return *this;
    }

    Resource set_path(const utility::string_t& path) {
        builder.set_path(path);
        return *this;
    }

    Resource append_path(const utility::string_t& path) {
        builder.append_path(path);
        return *this;
    }

    std::string to_string() {
        return builder.to_string();
    }
};

class HTTPInterface {
public:
    HTTPInterface(const utility::string_t& host,
                  const utility::string_t& path = "")
        :host(host), base_path(path){}

    const std::map<std::string, std::string> get(const std::vector<std::string> &json_keys,
                                           const std::string command,
                                           const std::initializer_list<param_type> &list = std::initializer_list<param_type>()) {


        const std::string uri = Resource(host).set_path(base_path).append_path(command).append_query(list).to_string();

        http_client client(uri);
        auto response = client.request(methods::GET).get();
        response.headers().set_content_type("application/json");
        json::value json_resp = response.extract_json().get();

        std::map<std::string, std::string> json_kv;
        for(std::string key : json_keys) {
            try{
                json_kv[key] = to_string(json_resp.at(U(key)));
            } catch (std::exception &e) {
                json_kv[key] = "--COULD NOT RETRIEVE VALUE--";
            }
        }
        return json_kv;
    }

private:
    const utility::string_t host;
    const utility::string_t base_path;
};
