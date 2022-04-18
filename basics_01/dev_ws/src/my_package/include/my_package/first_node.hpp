#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_package/visibility_control.h"
#include "my_interface/msg/num.hpp"
#include "my_interface/srv/add_three_ints.hpp"
#include "my_interface/action/sum.hpp"

namespace my_package
{

class FirstNode : public rclcpp::Node
{
private:
  using GoalHandleMyInterfaceSum = rclcpp_action::ServerGoalHandle<my_interface::action::Sum>;

public:
  MY_PACKAGE_PUBLIC explicit FirstNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();

  /* Service */
  void service_add3(const std::shared_ptr<my_interface::srv::AddThreeInts::Request> request, std::shared_ptr<my_interface::srv::AddThreeInts::Response> response);

  /* Action */
  rclcpp_action::GoalResponse action_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const my_interface::action::Sum::Goal> goal);
  rclcpp_action::CancelResponse action_handle_cancel(const std::shared_ptr<GoalHandleMyInterfaceSum> goal_handle);
  void action_handle_accepted(const std::shared_ptr<GoalHandleMyInterfaceSum> goal_handle);
  void action_execute(const std::shared_ptr<GoalHandleMyInterfaceSum> goal_handle);

private:
  /* Parameter */
  std::string parameter_string_;

  /* Topic */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_1_;
  rclcpp::Publisher<my_interface::msg::Num>::SharedPtr publisher_2_;

  /* Service */
  rclcpp::Service<my_interface::srv::AddThreeInts>::SharedPtr service_server_;

  /* Action */
  rclcpp_action::Server<my_interface::action::Sum>::SharedPtr action_server_;

  rclcpp::TimerBase::SharedPtr timer_;

  size_t count_;

};

}
