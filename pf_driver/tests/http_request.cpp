#include "pf_driver/http_helpers.hpp"
#include <gtest/gtest.h>
#include <iostream>

/*
// Declare a test
TEST(HHTPInterface_TestSuite, testURIBuilder)
{
    HTTPInterface http_interface("10.0.10.7", "cmd");

    const std::string err_code = "error_code";
    const std::string err_text = "error_text";
    const std::string err_http = "error_http";
    std::vector<std::string> err = { err_code, err_text };
    std::map<std::string, std::string> json_resp = http_interface.get(err, "get_protocol_info");

    EXPECT_NE(json_resp[err_http], std::string("OK"));
}
*/

/*
TEST(HHTPInterface_TestSuite, testURIBuilder2)
{
    HTTPInterface http_interface("10.0.10.76", "cmd");

    const std::string err_code = "error_code";
    const std::string err_text = "error_text";
    const std::string err_http = "error_http";
    std::vector<std::string> err = { err_code, err_text };
    std::map<std::string, std::string> json_resp = http_interface.get(err, "get_protocol_info");

    EXPECT_EQ(json_resp[err_http], std::string("OK"));
    EXPECT_EQ(json_resp[err_code], std::string("0"));
    EXPECT_EQ(json_resp[err_text], std::string("success"));
}
*/
