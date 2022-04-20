#include "rclcpp/rclcpp.hpp"
#include "my_package/visibility_control.h"

namespace my_package
{

class Listener : public rclcpp::Node
{

public:
  MY_PACKAGE_PUBLIC explicit Listener(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void topic_callback(const std_msgs::msg::Int32::SharedPtr msg);

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_1_;

};

}
