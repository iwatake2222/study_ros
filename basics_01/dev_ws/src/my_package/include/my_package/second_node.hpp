#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_package/visibility_control.h"
#include "my_interface/msg/num.hpp"
#include "my_interface/srv/add_three_ints.hpp"
#include "my_interface/action/sum.hpp"

namespace my_package
{

class SecondNode : public rclcpp::Node
{
private:
  using GoalHandleMyInterfaceSum = rclcpp_action::ClientGoalHandle<my_interface::action::Sum>;

public:
  MY_PACKAGE_PUBLIC explicit SecondNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /* Topic */
  void topic_callback_1(const std_msgs::msg::String::SharedPtr msg);
  void topic_callback_2(const my_interface::msg::Num::SharedPtr msg);

  /* Service */
  void service_callback(rclcpp::Client<my_interface::srv::AddThreeInts>::SharedFuture future);

  /* Action */
  void action_send_goal();
  void action_goal_response_callback(std::shared_future<GoalHandleMyInterfaceSum::SharedPtr> future);
  void action_feedback_callback(GoalHandleMyInterfaceSum::SharedPtr, const std::shared_ptr<const my_interface::action::Sum::Feedback> feedback);
  void action_result_callback(const GoalHandleMyInterfaceSum::WrappedResult & result);

private:
  /* Topic */
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_1_;
  rclcpp::Subscription<my_interface::msg::Num>::SharedPtr subscription_2_;

  /* Service */
  rclcpp::Client<my_interface::srv::AddThreeInts>::SharedPtr service_client_;

  /* Action */
  rclcpp_action::Client<my_interface::action::Sum>::SharedPtr action_client_;
};

}
