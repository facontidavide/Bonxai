#ifndef BONXAI_SERVER__BONXAI_SERVER_HPP_
#define BONXAI_SERVER__BONXAI_SERVER_HPP_

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "bonxai/bonxai.hpp"
#include "bonxai_map/pcl_utils.hpp"
#include "bonxai_map/probabilistic_map.hpp"
#include "message_filters/subscriber.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

namespace bonxai_server {

using sensor_msgs::msg::PointCloud2;

class BonxaiServer : public rclcpp::Node {
 public:
  using PCLPoint = pcl::PointXYZ;
  using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;
  using BonxaiT = Bonxai::ProbabilisticMap;
  using ResetSrv = std_srvs::srv::Empty;

  explicit BonxaiServer(const rclcpp::NodeOptions& node_options);

  bool resetSrv(
      const std::shared_ptr<ResetSrv::Request> req, const std::shared_ptr<ResetSrv::Response> resp);

  virtual void insertCloudCallback(const PointCloud2::ConstSharedPtr cloud);

 protected:
  virtual void publishAll(const rclcpp::Time& rostime);

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  rcl_interfaces::msg::SetParametersResult onParameter(
      const std::vector<rclcpp::Parameter>& parameters);

  rclcpp::Publisher<PointCloud2>::SharedPtr point_cloud_pub_;
  message_filters::Subscriber<PointCloud2> point_cloud_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<PointCloud2>> tf_point_cloud_sub_;
  // rclcpp::Service<BBoxSrv>::SharedPtr clear_bbox_srv_;
  rclcpp::Service<ResetSrv>::SharedPtr reset_srv_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  std::unique_ptr<BonxaiT> bonxai_;
  std::vector<Bonxai::CoordT> key_ray_;

  double max_range_;
  std::string world_frame_id_;  // the map frame
  std::string base_frame_id_;   // base of the robot for ground plane filtering

  bool latched_topics_;

  double res_;

  double occupancy_min_z_;
  double occupancy_max_z_;

  bool publish_2d_map_;
  bool map_origin_changed;
  // octomap::OcTreeKey padded_min_key_;
  unsigned multires_2d_scale_;
  bool project_complete_map_;
};
}  // namespace bonxai_server

#endif  // BONXAI_SERVER__BONXAI_SERVER_HPP_
