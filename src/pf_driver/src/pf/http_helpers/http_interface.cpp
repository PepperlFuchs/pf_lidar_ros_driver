#include "pf_driver/pf/http_helpers/http_interface.h"
#include "pf_driver/pf/http_helpers/curl_resource.h"
#include "pf_driver/pf/http_helpers/http_helpers.h"

HTTPInterface::HTTPInterface(std::string host, std::string path) : host(std::move(host)), base_path(std::move(path))
{
}

const std::map<std::string, std::string> HTTPInterface::get(const std::vector<std::string>& json_keys,
                                                            const std::string& command,
                                                            const std::initializer_list<param_type>& list)
{
  CurlResource res(host);
  res.append_path(base_path);
  res.append_path(command);
  res.append_query(list);
  return get_(json_keys, res);
}

const std::map<std::string, std::string> HTTPInterface::get(const std::vector<std::string>& json_keys,
                                                            const std::string& command, const param_map_type& params)
{
  CurlResource res(host);
  res.append_path(base_path);
  res.append_path(command);
  res.append_query(params);
  return get_(json_keys, res);
}

const std::map<std::string, std::string> HTTPInterface::get_(const std::vector<std::string>& json_keys,
                                                             CurlResource& res)
{
  Json::Value json_resp;
  std::map<std::string, std::string> json_kv;

  try
  {
    res.get(json_resp);
    json_kv[std::string("error_http")] = std::string("OK");
  }
  catch (curlpp::RuntimeError& e)
  {
    json_kv[std::string("error_http")] = std::string(e.what());
    return json_kv;
  }
  catch (curlpp::LogicError& e)
  {
    json_kv[std::string("error_http")] = std::string(e.what());
    return json_kv;
  }

  for (std::string key : json_keys)
  {
    try
    {
      if (json_resp[key].isArray())
        json_kv[key] = http_helpers::from_array(json_resp[key]);
      else
        json_kv[key] = json_resp[key].asString();
    }
    catch (std::exception& e)
    {
      json_kv[key] = "--COULD NOT RETRIEVE VALUE--";
    }
  }
  return json_kv;
}
