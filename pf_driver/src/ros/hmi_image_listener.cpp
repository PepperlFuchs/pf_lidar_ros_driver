#include "pf_driver/ros/hmi_image_listener.h"

#include <b64/encode.h>
#include <sensor_msgs/image_encodings.hpp>
#include "pf_driver/pf/pfsdp_protocol.hpp"

HmiImageListener::HmiImageListener(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<PFSDPBase> protocol)
  : node_(node)
  , subscriber_(node_->create_subscription<sensor_msgs::msg::Image>("/hmi_image",
                                                                    10,
                                                                    std::bind(&HmiImageListener::on_image_published,
                                                                              this,
                                                                              std::placeholders::_1)))
  , protocol_(protocol)
{
}

void HmiImageListener::on_image_published(sensor_msgs::msg::Image::SharedPtr image)
{
  const int byte_depth = sensor_msgs::image_encodings::bitDepth(image->encoding) / 8;
  const int channels = sensor_msgs::image_encodings::numChannels(image->encoding);

  const int scanner_image_width = 252;
  const int scanner_image_height = 24;
  const int scanner_image_height_bytes = scanner_image_height / 8;

  const uint32_t image_width = image->width;
  const uint32_t image_height = image->height;
  const uint32_t step = image->step;

  const size_t raw_data_size = scanner_image_width * scanner_image_height_bytes;
  char raw_data[raw_data_size];
  memset(raw_data, 0, raw_data_size);

  for(size_t x = 0 ; x < scanner_image_width && x < image_width ; ++x)
  {
    for(size_t y = 0 ; y < scanner_image_height && y < image_height ; ++y)
    {
      for(size_t channel = 0 ; channel < channels ; ++channel)
      {
        for(size_t byte = 0 ; byte < byte_depth ; ++byte)
        {
          if(image->data[y * step + x * channels * byte_depth + channel * byte_depth + byte])
          {
            size_t byte_index = (x * scanner_image_height_bytes) + (2 - y / 8);
            raw_data[byte_index] |= (1 << (y % 8));
          }
        }
      }
    }
  }

  std::istringstream input_stream(std::string(raw_data, raw_data_size));
  std::ostringstream output_stream;

  base64::encoder encoder;
  encoder.encode(input_stream, output_stream);

  std::string base64_str = output_stream.str();
  auto remove_end = std::remove(base64_str.begin(), base64_str.end(), '\n');
  base64_str.erase(remove_end, base64_str.end());

  // base64 -> base64url
  std::replace(base64_str.begin(), base64_str.end(), '+', '-');
  std::replace(base64_str.begin(), base64_str.end(), '/', '_');

  protocol_->set_parameter({ KV("hmi_application_bitmap", base64_str) });
}
