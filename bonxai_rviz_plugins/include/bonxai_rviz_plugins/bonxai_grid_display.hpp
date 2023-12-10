// 
// Author: John D'Angelo
//
// This plugin takes a lot of inspiration from the Octomap rviz plugin
// https://github.com/OctoMap/octomap_rviz_plugins/tree/ros2
// 

#ifndef BONXAI_RVIZ_PLUGINS__OCCUPANCY_GRID_DISPLAY_HPP_
#define BONXAI_RVIZ_PLUGINS__OCCUPANCY_GRID_DISPLAY_HPP_

#ifndef Q_MOC_RUN

#include <memory>
#include <string>
#include <vector>

#include "bonxai_msgs/msg/bonxai.hpp"

#include <std_msgs/msg/color_rgba.hpp>

#include "rviz_rendering/objects/point_cloud.hpp"
#include "rviz_common/message_filter_display.hpp"

#endif

namespace rviz_common
{
  namespace properties
  {
    class IntProperty;
    class EnumProperty;
    class FloatProperty;
  }
}

namespace bonxai_rviz_plugins
{

  class BonxaiGridDisplay
      : public rviz_common::MessageFilterDisplay<bonxai_msgs::msg::Bonxai>
  {
    Q_OBJECT

  public:
    BonxaiGridDisplay();

    // Overrides from Display
    void onInitialize() override;
    void update(float wall_dt, float ros_dt) override;
    void reset() override;

  private Q_SLOTS:
    void updateBonxaiColorMode();
    void updateAlpha();

  protected:
    void unsubscribe() override;

    void setColorFromZAxis(
        double z_pos, double color_factor,
        rviz_rendering::PointCloud::Point &point);

    void clear();

    virtual bool updateFromTF();

    std::mutex mutex_;

    // point buffer
    std::vector<rviz_rendering::PointCloud::Point> new_points_;
    std::vector<rviz_rendering::PointCloud::Point> point_buf_;
    bool new_points_received_{false};

    // Ogre-rviz point clouds
    std::shared_ptr<rviz_rendering::PointCloud> cloud_;
    double box_size_;
    std_msgs::msg::Header header_;

    // Plugin properties
    rviz_common::properties::EnumProperty *bonxai_coloring_property_;
    rviz_common::properties::FloatProperty *alpha_property_;

    // parameters to control the z-axis scaling
    double color_factor_{0.8};
    double max_z_{1.0};
    double min_z_{-1.0};

    // this is to make iterating over the faces of a cube more convenient
    const std::vector<std::vector<int32_t>> faces{
        {1, 0, 0},
        {-1, 0, 0},
        {0, 1, 0},
        {0, -1, 0},
        {0, 0, 1},
        {0, 0, -1},
    };
  };

  /**
   * @brief Templated version of BonxaiGrid (not meant to be used)
  */
  template <typename CellT>
  class TemplatedBonxaiGridDisplay : public BonxaiGridDisplay
  {
  protected:
    void processMessage(const bonxai_msgs::msg::Bonxai::ConstSharedPtr msg) override;

    virtual void setVoxelColor(
        rviz_rendering::PointCloud::Point &new_point,
        CellT &cell) = 0;

    /// Returns false, if the type_id (of the message) does not correspond to the template paramter
    /// of this class, true if correct or unknown (i.e., no specialized method for that template).
    virtual bool checkType(const std::string &type_id) = 0;
  };

  /**
   * @brief Specialization of BonxaiGrid template for scalar values (float, int, etc...)
  */
  template <typename CellT>
  class ScalarBonxaiGridDisplay : public TemplatedBonxaiGridDisplay<CellT>
  {
  protected:
    void setVoxelColor(
        rviz_rendering::PointCloud::Point &new_point,
        CellT &cell) override;

    bool checkType(const std::string &type_id) override;
  };

  /**
   * @brief Specialization of BonxaiGrid template for color values (RGB)
  */
  class ColorBonxaiGridDisplay : public TemplatedBonxaiGridDisplay<std_msgs::msg::ColorRGBA>
  {
  protected:
    void setVoxelColor(
        rviz_rendering::PointCloud::Point &new_point,
        std_msgs::msg::ColorRGBA &cell) override;

    bool checkType(const std::string &type_id) override;
  };

} // namespace bonxai_rviz_plugins

#endif // BONXAI_RVIZ_PLUGINS__OCCUPANCY_GRID_DISPLAY_HPP_