#include <cstdlib>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "my_package/second_node.hpp"

namespace my_package
{

SecondNode::SecondNode(const rclcpp::NodeOptions & options)
: Node("second_node", options)
{
  /* Topic: Subscriber */
  subscription_1_ = this->create_subscription<std_msgs::msg::String>("my_topic_1", 10, std::bind(&SecondNode::topic_callback_1, this, std::placeholders::_1));
  subscription_2_ = this->create_subscription<my_interface::msg::Num>("my_topic_2", 10, std::bind(&SecondNode::topic_callback_2, this, std::placeholders::_1));

  /* Service: Client */
  client_ = this->create_client<my_interface::srv::AddThreeInts>("add_three_ints");
  while (!client_->wait_for_service(std::chrono::milliseconds(500))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
}

void SecondNode::topic_callback_1(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "topic_callback_1: '%s'", msg->data.c_str());
}

void SecondNode::topic_callback_2(const my_interface::msg::Num::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "topic_callback_2: '%d'", msg->num);

    auto request = std::make_shared<my_interface::srv::AddThreeInts::Request>();
    request->a = msg->num;
    request->b = msg->num * 100;
    request->c = msg->num * 10000;
    auto result = client_->async_send_request(request, std::bind(&SecondNode::service_callback, this, std::placeholders::_1));
}

void SecondNode::service_callback(rclcpp::Client<my_interface::srv::AddThreeInts>::SharedFuture future)
  {
    RCLCPP_INFO(this->get_logger(), "service_callback: %d", future.get()->sum);
  }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_package::SecondNode)