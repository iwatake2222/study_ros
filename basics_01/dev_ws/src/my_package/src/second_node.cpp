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
  service_client_ = this->create_client<my_interface::srv::AddThreeInts>("my_service_add3");
  while (!service_client_->wait_for_service(std::chrono::milliseconds(500))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  /* Action: Client */
  this->action_client_ = rclcpp_action::create_client<my_interface::action::Sum>(this, "my_action_sum");

  /* Call Action */
  action_send_goal();
}

void SecondNode::topic_callback_1(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "topic_callback_1: '%s'", msg->data.c_str());
}

void SecondNode::topic_callback_2(const my_interface::msg::Num::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "topic_callback_2: '%d'", msg->num);

  /* Call Service */
  auto request = std::make_shared<my_interface::srv::AddThreeInts::Request>();
  request->a = msg->num;
  request->b = msg->num * 100;
  request->c = msg->num * 10000;
  auto result = service_client_->async_send_request(request, std::bind(&SecondNode::service_callback, this, std::placeholders::_1));
}

void SecondNode::service_callback(rclcpp::Client<my_interface::srv::AddThreeInts>::SharedFuture future)
{
  RCLCPP_INFO(this->get_logger(), "service_callback: %d", future.get()->sum);
}

void SecondNode::action_send_goal()
{
  if (!this->action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = my_interface::action::Sum::Goal();
  goal_msg.order = 10;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<my_interface::action::Sum>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&SecondNode::action_goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&SecondNode::action_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&SecondNode::action_result_callback, this, std::placeholders::_1);
  this->action_client_->async_send_goal(goal_msg, send_goal_options);
}

void SecondNode::action_goal_response_callback(std::shared_future<GoalHandleMyInterfaceSum::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "[action_goal_response_callback] Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "[action_goal_response_callback] Goal accepted by server, waiting for result");
  }
}

void SecondNode::action_feedback_callback(GoalHandleMyInterfaceSum::SharedPtr, const std::shared_ptr<const my_interface::action::Sum::Feedback> feedback)
{
  std::stringstream ss;
  ss << "[action_feedback_callback] ";
  for (auto number : feedback->partial_sequence) {
    ss << number << " ";
  }
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}

void SecondNode::action_result_callback(const GoalHandleMyInterfaceSum::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "[action_result_callback] Goal was succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "[action_result_callback] Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "[action_result_callback] Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "[action_result_callback] Unknown result code");
      return;
  }
  std::stringstream ss;
  ss << "Result received: ";
  for (auto number : result.result->sequence) {
    ss << number << " ";
  }
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_package::SecondNode)