#include "rclcpp/rclcpp.hpp"
#include "my_package/visibility_control.h"
#include "my_interface/msg/num.hpp"
#include "my_interface/srv/add_three_ints.hpp"
#include "my_interface/action/sum.hpp"

namespace my_package
{

class FirstNode : public rclcpp::Node
{
public:
  MY_PACKAGE_PUBLIC explicit FirstNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();
  void add3(const std::shared_ptr<my_interface::srv::AddThreeInts::Request> request, std::shared_ptr<my_interface::srv::AddThreeInts::Response> response);

private:
  std::string parameter_string_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_1_;
  rclcpp::Publisher<my_interface::msg::Num>::SharedPtr publisher_2_;
  rclcpp::Service<my_interface::srv::AddThreeInts>::SharedPtr service_;

  size_t count_;

};

}
