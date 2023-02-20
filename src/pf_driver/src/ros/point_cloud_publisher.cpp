#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pf_driver/PFR2300Header.h"
#include "pf_driver/ros/point_cloud_publisher.h"

PointcloudPublisher::PointcloudPublisher(std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params,
                                         const std::string& scan_topic, const std::string& frame_id,
                                         const uint16_t num_layers, const std::string& part)
  : PFDataPublisher(config, params), layer_prev_(-1)
{
  ros::NodeHandle p_nh("~/");

  XmlRpc::XmlRpcValue angles_param;
  p_nh.getParam("correction_params", angles_param);

  angles_.resize(num_layers);

  for (size_t i = 0; i < angles_param.size(); i++)
  {
    angles_[i] = angles_param[i]["ang"];
    auto coeff_param = angles_param[i]["coeff"];
    std::vector<double> coeffs;
    for (size_t j = 0; j < coeff_param.size(); j++)
    {
      coeffs.push_back(coeff_param[j]);
    }

    correction_params_[angles_[i]] = coeffs;
  }

  for (int i = 0; i < angles_param.size(); i++)
  {
    std::string topic = scan_topic + "_" + std::to_string(i + 1);
    std::string id = frame_id + "_" + std::to_string(i + 1);

    // init frames for each layer
    publish_static_transform(frame_id, id, angles_[i]);

    scan_publishers_.push_back(p_nh.advertise<sensor_msgs::LaserScan>(topic.c_str(), 100));
    frame_ids_.push_back(id);
  }

  cloud_.reset(new sensor_msgs::PointCloud2());
  pcl_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(scan_topic, 1);
  header_publisher_ = nh_.advertise<pf_driver::PFR2300Header>("/r2300_header", 1);
  frame_id_.assign(frame_id);
}

void PointcloudPublisher::resetCurrentScans()
{
  cloud_.reset(new sensor_msgs::PointCloud2());
  layer_prev_ = -1;
}

void PointcloudPublisher::publish_static_transform(const std::string& parent, const std::string& child,
                                                   int inclination_angle)
{
  geometry_msgs::TransformStamped transform;

  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = parent.c_str();
  transform.child_frame_id = child.c_str();
  transform.transform.translation.x = 0;
  transform.transform.translation.y = 0;
  transform.transform.translation.z = 0;

  tf2::Quaternion quat;
  double ang = inclination_angle / 10000.0 * M_PI / 180.0;
  quat.setRPY(0, ang, 0);
  transform.transform.rotation.x = quat.x();
  transform.transform.rotation.y = quat.y();
  transform.transform.rotation.z = quat.z();
  transform.transform.rotation.w = quat.w();

  static_broadcaster_.sendTransform(transform);
}

void PointcloudPublisher::publish_scan(sensor_msgs::LaserScanPtr msg, uint16_t idx)
{
  msg->header.frame_id = frame_ids_.at(idx);
  scan_publishers_.at(idx).publish(msg);
}

void PointcloudPublisher::handle_scan(sensor_msgs::LaserScanPtr msg, uint16_t layer_idx, int layer_inclination,
                                      bool apply_correction)
{
  publish_scan(msg, layer_idx);

  sensor_msgs::PointCloud2 c;
  int channelOptions = laser_geometry::channel_option::Intensity;
  if (apply_correction)
  {
    // since 'apply_correction' calculates the point cloud from laser scan message,
    // 'transformLaserScanToPointCloud' is not be needed anymore
    // just 'projectLaser' is enough just so that it initializes the pointcloud message correctly
    projector_.projectLaser(*msg, c);
    project_laser(c, msg, layer_inclination);
  }
  else
  {
    projector_.transformLaserScanToPointCloud(frame_id_, *msg, c, tfListener_, -1.0, channelOptions);
  }

  if (layer_idx <= layer_prev_)
  {
    if (!cloud_->data.empty())
    {
      cloud_->header.frame_id = frame_id_;
      pcl_publisher_.publish(cloud_);
      cloud_.reset(new sensor_msgs::PointCloud2());
    }
    copy_pointcloud(*cloud_, c);
  }
  else
  {
    add_pointcloud(*cloud_, c);
  }
  layer_prev_ = layer_idx;
}

void PointcloudPublisher::copy_pointcloud(sensor_msgs::PointCloud2& c1, sensor_msgs::PointCloud2 c2)
{
  c1.header.frame_id = c2.header.frame_id;
  c1.height = c2.height;
  c1.width = c2.width;
  c1.is_bigendian = c2.is_bigendian;
  c1.point_step = c2.point_step;
  c1.row_step = c2.row_step;

  c1.fields = std::move(c2.fields);
  c1.data = std::move(c2.data);
}

void PointcloudPublisher::add_pointcloud(sensor_msgs::PointCloud2& c1, sensor_msgs::PointCloud2 c2)
{
  pcl::PCLPointCloud2 p1, p2;
  pcl_conversions::toPCL(c1, p1);
  pcl::PointCloud<pcl::PointXYZI>::Ptr p1_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  // handle when point cloud is empty
  pcl::fromPCLPointCloud2(p1, *p1_cloud);

  pcl_conversions::toPCL(c2, p2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr p2_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(p2, *p2_cloud);

  *p1_cloud += *p2_cloud;
  pcl::toROSMsg(*p1_cloud.get(), c1);
}

void PointcloudPublisher::project_laser(sensor_msgs::PointCloud2& c, sensor_msgs::LaserScanPtr msg,
                                        const int layer_inclination)
{
  pcl::PCLPointCloud2 p;
  pcl_conversions::toPCL(c, p);
  pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  // handle when point cloud is empty
  pcl::fromPCLPointCloud2(p, *p_cloud);

  size_t cl_idx = 0;

  double angle_v_deg = layer_inclination / 10000.0;
  double angle_v = (M_PI / 180.0) * angle_v_deg;

  for (size_t i = 0; i < msg->ranges.size(); i++)
  {
    // num of points in cloud is sometimes less than that in laser scan because of
    // https://github.com/ros-perception/laser_geometry/blob/indigo-devel/src/laser_geometry.cpp#L110
    if (msg->ranges[i] < msg->range_min || msg->ranges[i] >= msg->range_max)
    {
      continue;
    }
    double angle_h = msg->angle_min + msg->angle_increment * (double)i;

    angle_v = correction_params_[layer_inclination][0] * angle_h * angle_h +
              correction_params_[layer_inclination][1] * angle_h + correction_params_[layer_inclination][2];

    p_cloud->points[cl_idx].x = cos(angle_h) * cos(angle_v) * msg->ranges[i];
    p_cloud->points[cl_idx].y = sin(angle_h) * cos(angle_v) * msg->ranges[i];
    p_cloud->points[cl_idx].z = sin(angle_v) * msg->ranges[i];

    cl_idx++;
  }

  pcl::toROSMsg(*p_cloud.get(), c);
}
