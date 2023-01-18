#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

class NavigationBT : public rclcpp::Node
{
public:
    NavigationBT() : Node("navigation_bt")
    {
        RCLCPP_INFO(this->get_logger(), "Starting NavigationBT node");

        this->declare_parameter<std::string>("xml_filepath");
        this->declare_parameter<std::vector<std::string>>("plugin_lib_names");
        this->declare_parameter<int>("tick_frequency_ms");

        this->get_parameter<std::string>("xml_filepath", xml_filepath_);
        this->get_parameter<std::vector<std::string>>("plugin_lib_names", plugin_lib_names_);
        this->get_parameter<int>("tick_frequency_ms", tick_frequency_ms_);

        RCLCPP_INFO(this->get_logger(), "Loading NavigationBT plugins");
        BT::SharedLibrary loader;
        for (const auto& p : plugin_lib_names_)
        {
            factory_.registerFromPlugin(loader.getOSName(p));
        }

        RCLCPP_INFO(this->get_logger(), "Loading NavigationBT structure from XML");

        blackboard_ = BT::Blackboard::create();
        blackboard_->set<rclcpp::Node::SharedPtr>("node", std::make_shared<rclcpp::Node>("bt_client_node"));
        blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(tick_frequency_ms_));
        blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(1000));

        tree_ = factory_.createTreeFromFile(xml_filepath_, blackboard_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(tick_frequency_ms_),
            std::bind(&NavigationBT::timerCallback, this));
    }

private:
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "ticking tree");
        tree_.tickRoot();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    BT::BehaviorTreeFactory factory_;
    BT::Blackboard::Ptr blackboard_;
    BT::Tree tree_;

    std::string xml_filepath_;
    std::vector<std::string> plugin_lib_names_;
    int tick_frequency_ms_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationBT>());
    rclcpp::shutdown();
    return 0;
}
