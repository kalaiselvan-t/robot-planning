#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>

#include "rclcpp/rclcpp.hpp"


#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.h"

// #include "tf2/exceptions.h"

// #include "rclcpp_action/rclcpp_action.hpp"
// #include "rclcpp_components/register_node_macro.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PositionListener : public rclcpp::Node
{
  public:
    PositionListener()
    : Node("get_positions")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("transform", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&PositionListener::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
      std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

      tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      geometry_msgs::msg::TransformStamped t;

      // Look up for the transformation between target_frame and turtle2 frames
      // and send velocity commands for turtle2 to reach target_frame
      try {
          rclcpp::Time now = this->get_clock()->now();
          t = tf_buffer_->lookupTransform("map", "shelfino1/base_link", tf2::TimePointZero, 20s);
      } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(this->get_logger(), "Could not transform map to base_link: %s", ex.what());
          return;
      }
      publisher_->publish(t);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr publisher_;
    size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionListener>());
  rclcpp::shutdown();
  return 0;
}