#include "bonxai_server.hpp"

namespace
{
template <typename T>
bool update_param(const std::vector<rclcpp::Parameter>& p,
                  const std::string& name,
                  T& value)
{
  auto it = std::find_if(
      p.cbegin(), p.cend(), [&name](const rclcpp::Parameter& parameter) {
        return parameter.get_name() == name;
      });
  if (it != p.cend())
  {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}
}  // namespace

namespace bonxai_server
{
BonxaiServer::BonxaiServer(const rclcpp::NodeOptions& node_options)
  : Node("bonxai_server_node", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  {
    world_frame_id_ = declare_parameter("frame_id", "map");
    base_frame_id_ = declare_parameter("base_frame_id", "base_footprint");
  }

  {
    rcl_interfaces::msg::ParameterDescriptor occupancy_min_z_desc;
    occupancy_min_z_desc.description =
        "Minimum height of occupied cells to consider in the final map";
    rcl_interfaces::msg::FloatingPointRange occupancy_min_z_range;
    occupancy_min_z_range.from_value = -100.0;
    occupancy_min_z_range.to_value = 100.0;
    occupancy_min_z_desc.floating_point_range.push_back(occupancy_min_z_range);
    occupancy_min_z_ =
        declare_parameter("occupancy_min_z", -100.0, occupancy_min_z_desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor occupancy_max_z_desc;
    occupancy_max_z_desc.description =
        "Maximum height of occupied cells to consider in the final map";
    rcl_interfaces::msg::FloatingPointRange occupancy_max_z_range;
    occupancy_max_z_range.from_value = -100.0;
    occupancy_max_z_range.to_value = 100.0;
    occupancy_max_z_desc.floating_point_range.push_back(occupancy_max_z_range);
    occupancy_max_z_ =
        declare_parameter("occupancy_max_z", 100.0, occupancy_max_z_desc);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor max_range_desc;
    max_range_desc.description = "Sensor maximum range";
    rcl_interfaces::msg::FloatingPointRange max_range_range;
    max_range_range.from_value = -1.0;
    max_range_range.to_value = 100.0;
    max_range_desc.floating_point_range.push_back(max_range_range);
    max_range_ = declare_parameter("sensor_model.max_range", -1.0, max_range_desc);
  }

  res_ = declare_parameter("resolution", 0.1);

  rcl_interfaces::msg::ParameterDescriptor prob_hit_desc;
  prob_hit_desc.description =
      "Probabilities for hits in the sensor model when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_hit_range;
  prob_hit_range.from_value = 0.5;
  prob_hit_range.to_value = 1.0;
  prob_hit_desc.floating_point_range.push_back(prob_hit_range);
  const double prob_hit = declare_parameter("sensor_model.hit", 0.7, prob_hit_desc);

  rcl_interfaces::msg::ParameterDescriptor prob_miss_desc;
  prob_miss_desc.description =
      "Probabilities for misses in the sensor model when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_miss_range;
  prob_miss_range.from_value = 0.0;
  prob_miss_range.to_value = 0.5;
  prob_miss_desc.floating_point_range.push_back(prob_miss_range);
  const double prob_miss =
      declare_parameter("sensor_model.miss", 0.4, prob_miss_desc);

  rcl_interfaces::msg::ParameterDescriptor prob_min_desc;
  prob_min_desc.description =
      "Minimum probability for clamping when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_min_range;
  prob_min_range.from_value = 0.0;
  prob_min_range.to_value = 1.0;
  prob_min_desc.floating_point_range.push_back(prob_min_range);
  const double thres_min =
      declare_parameter("sensor_model.min", 0.12, prob_min_desc);

  rcl_interfaces::msg::ParameterDescriptor prob_max_desc;
  prob_max_desc.description =
      "Maximum probability for clamping when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_max_range;
  prob_max_range.from_value = 0.0;
  prob_max_range.to_value = 1.0;
  prob_max_desc.floating_point_range.push_back(prob_max_range);
  const double thres_max =
      declare_parameter("sensor_model.max", 0.97, prob_max_desc);

  // initialize bonxai object & params
  RCLCPP_INFO(get_logger(), "Voxel resolution %f", res_);
  bonxai_ = std::make_unique<BonxaiT>(res_);
  BonxaiT::Options options = { bonxai_->logods(prob_miss),
                               bonxai_->logods(prob_hit),
                               bonxai_->logods(thres_min),
                               bonxai_->logods(thres_max) };
  bonxai_->setOptions(options);

  latched_topics_ = declare_parameter("latch", true);
  if (latched_topics_)
  {
    RCLCPP_INFO(get_logger(),
                "Publishing latched (single publish will take longer, "
                "all topics are prepared)");
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "Publishing non-latched (topics are only prepared as needed, "
                "will only be re-published on map change");
  }

  auto qos = latched_topics_ ? rclcpp::QoS{ 1 }.transient_local() : rclcpp::QoS{ 1 };
  point_cloud_pub_ =
      create_publisher<PointCloud2>("bonxai_point_cloud_centers", qos);

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  using std::chrono_literals::operator""s;
  point_cloud_sub_.subscribe(this, "cloud_in", rmw_qos_profile_sensor_data);
  tf_point_cloud_sub_ = std::make_shared<tf2_ros::MessageFilter<PointCloud2>>(
      point_cloud_sub_,
      *tf2_buffer_,
      world_frame_id_,
      5,
      this->get_node_logging_interface(),
      this->get_node_clock_interface(),
      5s);

  tf_point_cloud_sub_->registerCallback(&BonxaiServer::insertCloudCallback, this);

  reset_srv_ = create_service<ResetSrv>(
      "~/reset", std::bind(&BonxaiServer::resetSrv, this, _1, _2));

  // set parameter callback
  set_param_res_ = this->add_on_set_parameters_callback(
      std::bind(&BonxaiServer::onParameter, this, _1));
}

void BonxaiServer::insertCloudCallback(const PointCloud2::ConstSharedPtr cloud)
{
  const auto start_time = rclcpp::Clock{}.now();

  PCLPointCloud pc;  // input cloud for filtering and ground-detection
  pcl::fromROSMsg(*cloud, pc);

  // Sensor In Global Frames Coordinates
  geometry_msgs::msg::TransformStamped sensor_to_world_transform_stamped;
  try
  {
    sensor_to_world_transform_stamped =
        tf2_buffer_->lookupTransform(world_frame_id_,
                                     cloud->header.frame_id,
                                     cloud->header.stamp,
                                     rclcpp::Duration::from_seconds(1.0));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }

  Eigen::Matrix4f sensor_to_world =
      tf2::transformToEigen(sensor_to_world_transform_stamped.transform)
          .matrix()
          .cast<float>();

  // Transforming Points to Global Reference Frame
  pcl::transformPointCloud(pc, pc, sensor_to_world);

  // Getting the Translation from the sensor to the Global Reference Frame
  const auto& t = sensor_to_world_transform_stamped.transform.translation;

  const pcl::PointXYZ sensor_to_world_vec3(t.x, t.y, t.z);

  bonxai_->insertPointCloud(pc.points, sensor_to_world_vec3, 30.0);

  double total_elapsed = (rclcpp::Clock{}.now() - start_time).seconds();
  RCLCPP_DEBUG(
      get_logger(), "Pointcloud insertion in Bonxai done, %f sec)", total_elapsed);

  publishAll(cloud->header.stamp);
}

rcl_interfaces::msg::SetParametersResult
BonxaiServer::onParameter(const std::vector<rclcpp::Parameter>& parameters)
{
  update_param(parameters, "occupancy_min_z", occupancy_min_z_);
  update_param(parameters, "occupancy_max_z", occupancy_max_z_);

  double sensor_model_min{ get_parameter("sensor_model.min").as_double() };
  update_param(parameters, "sensor_model.min", sensor_model_min);
  double sensor_model_max{ get_parameter("sensor_model.max").as_double() };
  update_param(parameters, "sensor_model.max", sensor_model_max);
  double sensor_model_hit{ get_parameter("sensor_model.hit").as_double() };
  update_param(parameters, "sensor_model.hit", sensor_model_hit);
  double sensor_model_miss{ get_parameter("sensor_model.miss").as_double() };
  update_param(parameters, "sensor_model.miss", sensor_model_miss);

  BonxaiT::Options options = { bonxai_->logods(sensor_model_miss),
                               bonxai_->logods(sensor_model_hit),
                               bonxai_->logods(sensor_model_min),
                               bonxai_->logods(sensor_model_max) };

  bonxai_->setOptions(options);

  publishAll(now());

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void BonxaiServer::publishAll(const rclcpp::Time& rostime)
{
  const auto start_time = rclcpp::Clock{}.now();
  thread_local std::vector<Eigen::Vector3d> bonxai_result;
  bonxai_result.clear();
  bonxai_->getOccupiedVoxels(bonxai_result);

  if (bonxai_result.size() <= 1)
  {
    RCLCPP_WARN(get_logger(), "Nothing to publish, bonxai is empty");
    return;
  }

  bool publish_point_cloud =
      (latched_topics_ ||
       point_cloud_pub_->get_subscription_count() +
       point_cloud_pub_->get_intra_process_subscription_count() > 0);

  // init pointcloud for occupied space:
  if (publish_point_cloud)
  {
    thread_local pcl::PointCloud<PCLPoint> pcl_cloud;
    pcl_cloud.clear();

    for (const auto& voxel : bonxai_result)
    {
      if(voxel.z() >= occupancy_min_z_ && voxel.z() <= occupancy_max_z_)
      {
        pcl_cloud.push_back(PCLPoint(voxel.x(), voxel.y(), voxel.z()));
      }
    }
    PointCloud2 cloud;
    pcl::toROSMsg(pcl_cloud, cloud);

    cloud.header.frame_id = world_frame_id_;
    cloud.header.stamp = rostime;
    point_cloud_pub_->publish(cloud);
    RCLCPP_WARN(get_logger(), "Published occupancy grid with %ld voxels", pcl_cloud.points.size());
  }
}

bool BonxaiServer::resetSrv(const std::shared_ptr<ResetSrv::Request>,
                            const std::shared_ptr<ResetSrv::Response>)
{
  const auto rostime = now();
  bonxai_ = std::make_unique<BonxaiT>(res_);

  RCLCPP_INFO(get_logger(), "Cleared Bonxai");
  publishAll(rostime);

  return true;
}

}  // namespace bonxai_server

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(bonxai_server::BonxaiServer)
