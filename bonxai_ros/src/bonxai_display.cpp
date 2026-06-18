#include "bonxai_display.hpp"

namespace bonxai_ros {

BonxaiDisplay::BonxaiDisplay() {
  color_property_ = new rviz_common::properties::ColorProperty(
      "Color", QColor(0, 255, 0), "Color of the voxels.", this, SLOT(updateStyle()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
      "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this, SLOT(updateStyle()));

  dynamic_range_property_ = new rviz_common::properties::BoolProperty(
      "Dynamic Range", false, "Scale coloring based on the maximum Z in the data.", this,
      SLOT(updateStyle()));

  color_mode_property_ = new rviz_common::properties::EnumProperty(
      "Color Mode", "Z Axis", "Select coloring mode.", this, SLOT(updateStyle()));
  color_mode_property_->addOption("Fixed Color", 0);
  color_mode_property_->addOption("X Axis", 1);
  color_mode_property_->addOption("Y Axis", 2);
  color_mode_property_->addOption("Z Axis", 3);

  min_value_property_ = new rviz_common::properties::FloatProperty(
      "Min Value", 0.0, "Minimum value for color scaling.", this, SLOT(updateStyle()));

  max_value_property_ = new rviz_common::properties::FloatProperty(
      "Max Value", 3.0, "Maximum value for color scaling.", this, SLOT(updateStyle()));
}

BonxaiDisplay::~BonxaiDisplay() {
  cloud_.reset();
}

void BonxaiDisplay::onInitialize() {
  MFDClass::onInitialize();
  cloud_ = std::make_unique<rviz_rendering::PointCloud>();
  cloud_->setRenderMode(rviz_rendering::PointCloud::RM_BOXES);
  scene_node_->attachObject(cloud_.get());
}

void BonxaiDisplay::processMessage(bonxai_ros::msg::BonxaiVoxelMap::ConstSharedPtr message) {
  last_msg_ = message;
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(message->header, position, orientation)) {
    return;
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  double res = message->voxel_resolution;
  uint32_t num_pts = message->occupied_voxels.size();

  RCLCPP_DEBUG(rclcpp::get_logger("rviz2"), "BonxaiDisplay: Received %u points", num_pts);
  setStatus(
      rviz_common::properties::StatusProperty::Ok, "Points",
      QString::number(num_pts) + " points received");

  cloud_->clear();
  if (num_pts == 0) {
    return;
  }

  cloud_->setDimensions(res, res, res);

  Ogre::ColourValue fixed_color = color_property_->getOgreColor();
  float alpha = alpha_property_->getFloat();
  fixed_color.a = alpha;

  cloud_->setAlpha(alpha);

  int mode = color_mode_property_->getOptionInt();
  float min_val = min_value_property_->getFloat();
  float max_val = max_value_property_->getFloat();


  if (dynamic_range_property_->getBool() && num_pts > 0) {
    float sensor_max_z = std::numeric_limits<float>::min();
    float sensor_min_z = std::numeric_limits<float>::max();
    for (uint32_t i = 0; i < num_pts; ++i) {
      float z = message->occupied_voxels[i].z * res;
      if (z > sensor_max_z)
        sensor_max_z = z;
      if (z < sensor_min_z)
        sensor_min_z = z;
    }
    max_val = sensor_max_z;
    min_val = sensor_min_z;
  }
  

  if (max_val == min_val)
    max_val = min_val + 0.001f;

  std::vector<rviz_rendering::PointCloud::Point> points;
  points.reserve(num_pts);

  for (uint32_t i = 0; i < num_pts; ++i) {
    int32_t vx = message->occupied_voxels[i].x;
    int32_t vy = message->occupied_voxels[i].y;
    int32_t vz = message->occupied_voxels[i].z;

    rviz_rendering::PointCloud::Point p;
    p.position = Ogre::Vector3(vx * res, vy * res, vz * res);

    if (mode > 0) {
      float val = (mode == 1) ? p.position.x : (mode == 2) ? p.position.y : p.position.z;
      float norm = std::max(0.0f, std::min(1.0f, (val - min_val) / (max_val - min_val)));
      p.color.setHSB(0.66f * (1.0f - norm), 1.0f, 1.0f);
      p.color.a = alpha;
    } else {
      p.color = fixed_color;
    }

    points.push_back(p);
  }

  cloud_->addPoints(points.begin(), points.end());
}

void BonxaiDisplay::updateStyle() {
  if (last_msg_) {
    processMessage(last_msg_);
  }
}

}  // namespace bonxai_ros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(bonxai_ros::BonxaiDisplay, rviz_common::Display)
