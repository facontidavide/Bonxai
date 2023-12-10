/**
 * This executable creates a subclass of Node to subscribe to
 * Bonxai messages.
 * https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
 */

#include <bonxai/bonxai.hpp>
#include <bonxai_msgs/conversion.hpp>
#include <bonxai_msgs/msg/bonxai.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
        : Node("bonxai_subscriber")
    {
        scalar_subscription_ = this->create_subscription<bonxai_msgs::msg::Bonxai>("bonxai/scalar", 1, [this](bonxai_msgs::msg::Bonxai::ConstSharedPtr msg)
                                                                                   {
            std::string grid_type; // this will be CellT as a string
            Bonxai::VoxelGrid<float> grid = bonxai_msgs::fromRosMsg<float>(*msg, grid_type);
            RCLCPP_INFO(get_logger(), "Received bonxai of res: %f, size: %ld, and type: %s", grid.resolution, msg->raw_data.size(), grid_type.c_str()); });

        color_subscription_ = this->create_subscription<bonxai_msgs::msg::Bonxai>("bonxai/color", 1, [this](bonxai_msgs::msg::Bonxai::ConstSharedPtr msg)
                                                                                  {
            std::string grid_type; // this will be CellT as a string
            Bonxai::VoxelGrid<std_msgs::msg::ColorRGBA> grid = bonxai_msgs::fromRosMsg<std_msgs::msg::ColorRGBA>(*msg, grid_type);
            RCLCPP_INFO(get_logger(), "Received bonxai of res: %f, size: %ld, and type: %s", grid.resolution, msg->raw_data.size(), grid_type.c_str()); });
    }

private:
    rclcpp::Subscription<bonxai_msgs::msg::Bonxai>::SharedPtr scalar_subscription_;
    rclcpp::Subscription<bonxai_msgs::msg::Bonxai>::SharedPtr color_subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}