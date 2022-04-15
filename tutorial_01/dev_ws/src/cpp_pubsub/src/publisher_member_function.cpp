#include <cstdlib>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tutorial_interfaces/msg/num.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "tutorial_interfaces/srv/add_three_ints.hpp" 

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    /* Parameter */
    this->declare_parameter<std::string>("my_parameter", "world");

    /* Topic: Publisher */
    publisher_1_ = this->create_publisher<std_msgs::msg::String>("topic_1", 10);
    publisher_2_ = this->create_publisher<tutorial_interfaces::msg::Num>("topic_2", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));

    /* Service: Server */
    service_1_ = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", std::bind(&MinimalPublisher::add2, this, std::placeholders::_1, std::placeholders::_2));
    service_2_ = this->create_service<tutorial_interfaces::srv::AddThreeInts>("add_three_ints", std::bind(&MinimalPublisher::add3, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void timer_callback()
  {
    this->get_parameter("my_parameter", parameter_string_);
    RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());

    auto msg1 = std_msgs::msg::String();
    msg1.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing_1: '%s'", msg1.data.c_str());
    publisher_1_->publish(msg1);

    auto msg2 = tutorial_interfaces::msg::Num();
    msg2.num = count_ + 1000;
    RCLCPP_INFO(this->get_logger(), "Publishing_2: '%d'", msg2.num);
    publisher_2_->publish(msg2);
  }

  void add2(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
            std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
  {
    response->sum = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld", request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
  }

  void add3(const std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request,
            std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response>      response)
  {
    response->sum = request->a + request->b + request->c;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld" " c: %ld", request->a, request->b, request->c);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::string parameter_string_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_1_;
  rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_2_;
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_1_;
  rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service_2_;
  size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
