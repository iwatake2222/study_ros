#include <cstdlib>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "my_package/first_node.hpp"

namespace my_package
{

FirstNode::FirstNode(const rclcpp::NodeOptions & options)
: Node("first_node", options)
{
  /* Parameter */
  this->declare_parameter<std::string>("my_parameter", "I am First Node. ");
  this->get_parameter("my_parameter", parameter_string_);
  RCLCPP_INFO(this->get_logger(), "my_parameter: %s", parameter_string_.c_str());
  
  /* Topic: Publisher */
  publisher_1_ = this->create_publisher<std_msgs::msg::String>("my_topic_1", 10);
  publisher_2_ = this->create_publisher<my_interface::msg::Num>("my_topic_2", 10);

  /* Service: Server */
  service_ = this->create_service<my_interface::srv::AddThreeInts>("add_three_ints", std::bind(&FirstNode::add3, this, std::placeholders::_1, std::placeholders::_2));

  /* Timer */
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&FirstNode::timer_callback, this));
}

void FirstNode::timer_callback()
{
  /* Topic: Publisher */
  auto msg_1 = std_msgs::msg::String();
  msg_1.data = parameter_string_ + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing_1: '%s'", msg_1.data.c_str());
  publisher_1_->publish(msg_1);

  auto msg2 = my_interface::msg::Num();
  msg2.num = count_ * 2;
  RCLCPP_INFO(this->get_logger(), "Publishing_2: '%d'", msg2.num);
  publisher_2_->publish(msg2);
}

void FirstNode::add3(const std::shared_ptr<my_interface::srv::AddThreeInts::Request> request, std::shared_ptr<my_interface::srv::AddThreeInts::Response> response)
{
  response->sum = request->a + request->b + request->c;
  RCLCPP_INFO(this->get_logger(), "Incoming request: a: %ld" " b: %ld" " c: %ld", request->a, request->b, request->c);
  RCLCPP_INFO(this->get_logger(), "sending back response: [%ld]", (long int)response->sum);
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_package::FirstNode)