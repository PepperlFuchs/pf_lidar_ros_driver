#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

class PFSDPBase;

class HmiImageListener
{
  public:
    HmiImageListener(std::shared_ptr<PFSDPBase> protocol);


  private:
    void on_image_published(sensor_msgs::ImagePtr image);

    ros::NodeHandle nh_;
    ros::Subscriber hmi_image_subscriber_;
    std::shared_ptr<PFSDPBase> protocol_;
};

