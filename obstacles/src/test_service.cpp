// #include <chrono>
// #include <memory>

// #include "rclcpp/rclcpp.hpp"
// #include "service_interface/msg/num.hpp"                                            // CHANGE

// using namespace std::chrono_literals;

// class MinimalPublisher : public rclcpp::Node
// {
// public:
//   MinimalPublisher()
//   : Node("three_ints_pub"), count_(0)
//   {
//     publisher_ = this->create_publisher<service_interface::msg::Num>("three_ints", 10);  // CHANGE
//     timer_ = this->create_wall_timer(
//       500ms, std::bind(&MinimalPublisher::timer_callback, this));
//   }

// private:
//   void timer_callback()
//   {
//     auto message = service_interface::msg::Num();                                   // CHANGE
//     message.num = this->count_++;                                                     // CHANGE
//     RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.num << "'");    // CHANGE
//     publisher_->publish(message);
//   }
//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::Publisher<service_interface::msg::Num>::SharedPtr publisher_;             // CHANGE
//   size_t count_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MinimalPublisher>());
//   rclcpp::shutdown();
//   return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "service_interface/srv/add_three_ints.hpp"                                        // CHANGE

#include <memory>

void add(const std::shared_ptr<service_interface::srv::AddThreeInts::Request> request,     // CHANGE
          std::shared_ptr<service_interface::srv::AddThreeInts::Response> response)  // CHANGE
{
  response->sum = request->a + request->b + request->c;                                      // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld" " c: %ld",  // CHANGE
                request->a, request->b, request->c);                                         // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");   // CHANGE

  rclcpp::Service<service_interface::srv::AddThreeInts>::SharedPtr service =               // CHANGE
    node->create_service<service_interface::srv::AddThreeInts>("add_three_ints",  &add);   // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");                     // CHANGE

  rclcpp::spin(node);
  rclcpp::shutdown();
}