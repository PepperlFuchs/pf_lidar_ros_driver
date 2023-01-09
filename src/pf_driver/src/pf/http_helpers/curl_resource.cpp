#include "pf_driver/pf/http_helpers/curl_resource.h"

CurlResource::CurlResource(const std::string& host) : url_("http://" + host)
{
  header_.push_back("Content-Type: application/json");
  request_.setOpt(new curlpp::options::HttpHeader(header_));
  request_.setOpt(curlpp::options::WriteStream(&response_));
}

void CurlResource::append_path(const std::string& path)
{
  url_ += "/" + path;
}

void CurlResource::append_query(const std::initializer_list<param_type>& list, bool do_encoding)
{
  url_ += "?";
  for (const auto& p : list)
  {
    url_ += p.first + "=" + p.second + "&";
  }
  url_.pop_back();
}

void CurlResource::append_query(const param_map_type& params, bool do_encoding)
{
  url_ += "?";
  for (const auto& p : params)
  {
    url_ += p.first + "=" + p.second + "&";
  }
  url_.pop_back();
}

void CurlResource::get(Json::Value& json_resp)
{
  request_.setOpt(curlpp::options::Url(url_));
  request_.perform();

  Json::Reader reader;
  reader.parse(response_, json_resp);
}

void CurlResource::print()
{
  std::cout << url_ << std::endl;
}
