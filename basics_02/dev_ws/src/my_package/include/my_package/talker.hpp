#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "my_package/visibility_control.h"

namespace my_package
{

class Talker : public rclcpp::Node
{

public:
  MY_PACKAGE_PUBLIC explicit Talker(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;

};

}
