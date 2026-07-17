#ifndef BONXAI_RVIZ_PLUGIN__BONXAI_DISPLAY_HPP_
#define BONXAI_RVIZ_PLUGIN__BONXAI_DISPLAY_HPP_

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_rendering/objects/point_cloud.hpp>
#include <vector>

#include "bonxai_ros/msg/bonxai_voxel_map.hpp"

namespace bonxai_ros {
class BonxaiDisplay : public rviz_common::MessageFilterDisplay<bonxai_ros::msg::BonxaiVoxelMap> {
  Q_OBJECT
 public:
  BonxaiDisplay();
  ~BonxaiDisplay();

 private Q_SLOTS:
  void updateStyle();

 protected:
  void onInitialize() override;
  void processMessage(bonxai_ros::msg::BonxaiVoxelMap::ConstSharedPtr msg) override;

  std::unique_ptr<rviz_rendering::PointCloud> cloud_;
  rviz_common::properties::BoolProperty* dynamic_range_property_;
  rviz_common::properties::ColorProperty* color_property_;
  rviz_common::properties::FloatProperty* alpha_property_;
  rviz_common::properties::EnumProperty* color_mode_property_;
  rviz_common::properties::FloatProperty* min_value_property_;
  rviz_common::properties::FloatProperty* max_value_property_;
  bonxai_ros::msg::BonxaiVoxelMap::ConstSharedPtr last_msg_;
};
}  // namespace bonxai_ros

#endif  // BONXAI_RVIZ_PLUGIN__BONXAI_DISPLAY_HPP_