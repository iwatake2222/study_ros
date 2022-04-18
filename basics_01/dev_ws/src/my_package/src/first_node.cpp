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
  service_server_ = this->create_service<my_interface::srv::AddThreeInts>("my_service_add3", std::bind(&FirstNode::service_add3, this, std::placeholders::_1, std::placeholders::_2));

  /* Action: Server */
  this->action_server_ = rclcpp_action::create_server<my_interface::action::Sum>(
    this,
    "my_action_sum",
    std::bind(&FirstNode::action_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&FirstNode::action_handle_cancel, this, std::placeholders::_1),
    std::bind(&FirstNode::action_handle_accepted, this, std::placeholders::_1)
  );

  /* Timer */
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&FirstNode::timer_callback, this));
}

void FirstNode::timer_callback()
{
  auto msg_1 = std_msgs::msg::String();
  msg_1.data = parameter_string_ + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "[timer_callback] Publishing_1: '%s'", msg_1.data.c_str());
  publisher_1_->publish(msg_1);

  auto msg2 = my_interface::msg::Num();
  msg2.num = count_ * 2;
  RCLCPP_INFO(this->get_logger(), "[timer_callback] Publishing_2: '%d'", msg2.num);
  publisher_2_->publish(msg2);
}

void FirstNode::service_add3(const std::shared_ptr<my_interface::srv::AddThreeInts::Request> request, std::shared_ptr<my_interface::srv::AddThreeInts::Response> response)
{
  response->sum = request->a + request->b + request->c;
  RCLCPP_INFO(this->get_logger(), "[service_add3] a: %ld" " b: %ld" " c: %ld", request->a, request->b, request->c);
  RCLCPP_INFO(this->get_logger(), "[service_add3] sum: %ld", (long int)response->sum);
}

rclcpp_action::GoalResponse FirstNode::action_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const my_interface::action::Sum::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "[action_handle_goal] order: %d", goal->order);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FirstNode::action_handle_cancel(const std::shared_ptr<GoalHandleMyInterfaceSum> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "[action_handle_cancel]");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FirstNode::action_handle_accepted(const std::shared_ptr<GoalHandleMyInterfaceSum> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "[action_handle_accepted] %d", goal_handle->get_goal()->order);
  std::thread{std::bind(&FirstNode::action_execute, this, std::placeholders::_1), goal_handle}.detach();
}

void FirstNode::action_execute(const std::shared_ptr<GoalHandleMyInterfaceSum> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "[action_execute]");
  rclcpp::Rate loop_rate(1);

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<my_interface::action::Sum::Feedback>();
  auto & sequence = feedback->partial_sequence;
  sequence.push_back(0);

  auto result = std::make_shared<my_interface::action::Sum::Result>();

  for (int i = 1; (i <= goal->order) && rclcpp::ok(); i++) {
    if (goal_handle->is_canceling()) {
      result->sequence = sequence;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "[Goal canceled]");
      return;
    }
    sequence.push_back(sequence[i - 1] + i);

    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "[Publish feedback]");

    loop_rate.sleep();
  }

  if (rclcpp::ok()) {
    result->sequence = sequence;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "[Goal succeeded]");
  }
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_package::FirstNode)