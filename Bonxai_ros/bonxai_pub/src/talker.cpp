/**
 * This executable creates a subclass of Node to publish
 * Bonxai messages.
 * https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
 */

#include <random>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <bonxai/bonxai.hpp>
#include <bonxai_msgs/conversion.hpp>
#include <bonxai_msgs/msg/bonxai.hpp>

using namespace std::chrono_literals;

std::random_device rd;   // obtain a random number from hardware
std::mt19937 gen(rd());  // seed the generator
std::uniform_real_distribution<> pos_dist(-1, 1);  // define the range
std::uniform_real_distribution<> prob_dist(0, 1);  // define the range

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
    : Node("bonxai_publisher")
    , num_points_(0)
  {
    scalar_publisher_ =
        this->create_publisher<bonxai_msgs::msg::Bonxai>("bonxai/scalar", 1);
    color_publisher_ =
        this->create_publisher<bonxai_msgs::msg::Bonxai>("bonxai/color", 1);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // Create this accessor once, and reuse it as much as possible.
    auto scalar_accessor = scalar_grid_.createAccessor();
    auto color_accessor = color_grid_.createAccessor();

    // pick a random 3d point
    double x = pos_dist(gen);
    double y = pos_dist(gen);
    double z = pos_dist(gen);

    // pick a random value from 0 to 1
    float probability = prob_dist(gen);
    scalar_accessor.setValue(scalar_grid_.posToCoord(x, y, z), probability);

    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    color.r = prob_dist(gen);
    color.g = prob_dist(gen);
    color.b = prob_dist(gen);
    color_accessor.setValue(color_grid_.posToCoord(x, y, z), color);

    RCLCPP_INFO(this->get_logger(), "Publishing tree with size: %ld", num_points_);
    bonxai_msgs::msg::Bonxai scalar_message;
    bonxai_msgs::toRosMsg(scalar_grid_, scalar_message);
    scalar_message.header.frame_id = "map";
    scalar_message.header.stamp = this->get_clock()->now();

    bonxai_msgs::msg::Bonxai color_message;
    bonxai_msgs::toRosMsg(color_grid_, color_message);
    color_message.header.frame_id = "map";
    color_message.header.stamp = this->get_clock()->now();

    scalar_publisher_->publish(scalar_message);
    color_publisher_->publish(color_message);
    num_points_++;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<bonxai_msgs::msg::Bonxai>::SharedPtr scalar_publisher_,
      color_publisher_;
  Bonxai::VoxelGrid<float> scalar_grid_{ 0.1 };
  Bonxai::VoxelGrid<std_msgs::msg::ColorRGBA> color_grid_{ 0.05 };
  size_t num_points_ = 0;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}