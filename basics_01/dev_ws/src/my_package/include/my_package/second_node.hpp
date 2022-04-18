#include "rclcpp/rclcpp.hpp"
#include "my_package/visibility_control.h"
#include "my_interface/msg/num.hpp"
#include "my_interface/srv/add_three_ints.hpp"
#include "my_interface/action/sum.hpp"

namespace my_package
{

class SecondNode : public rclcpp::Node
{
public:
  MY_PACKAGE_PUBLIC explicit SecondNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void topic_callback_1(const std_msgs::msg::String::SharedPtr msg);
  void topic_callback_2(const my_interface::msg::Num::SharedPtr msg);
  void service_callback(rclcpp::Client<my_interface::srv::AddThreeInts>::SharedFuture future);

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_1_;
  rclcpp::Subscription<my_interface::msg::Num>::SharedPtr subscription_2_;
  rclcpp::Client<my_interface::srv::AddThreeInts>::SharedPtr client_;

};

}
