#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>

class PFSDPBase;

class HmiImageListener
{
  public:
    HmiImageListener(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<PFSDPBase> protocol);


  private:
    void on_image_published(sensor_msgs::msg::Image::SharedPtr image);

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    std::shared_ptr<PFSDPBase> protocol_;
};

