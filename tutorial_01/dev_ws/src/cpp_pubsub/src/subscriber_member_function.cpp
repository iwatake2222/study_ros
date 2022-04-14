#include <cstdlib>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tutorial_interfaces/msg/num.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "tutorial_interfaces/srv/add_three_ints.hpp" 

using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    /* Topic: Subscriber */
    subscription_1_ = this->create_subscription<std_msgs::msg::String>("topic_1", 10, std::bind(&MinimalSubscriber::topic_callback_1, this, std::placeholders::_1));
    subscription_2_ = this->create_subscription<tutorial_interfaces::msg::Num>("topic_2", 10, std::bind(&MinimalSubscriber::topic_callback_2, this, std::placeholders::_1));

    /* Service: Client */
    client_1_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    while (!client_1_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    client_2_ = this->create_client<tutorial_interfaces::srv::AddThreeInts>("add_three_ints");
    while (!client_2_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
  }

private:
  void topic_callback_1(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard from topic_1: '%s'", msg->data.c_str());
  }

  void topic_callback_2(const tutorial_interfaces::msg::Num::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard from topic_2: '%d'", msg->num);

    auto request_1 = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request_1->a = msg->num;
    request_1->b = msg->num * 10;
    auto result_1 = client_1_->async_send_request(request_1, std::bind(&MinimalSubscriber::response_callback_1, this, std::placeholders::_1));

    auto request_2 = std::make_shared<tutorial_interfaces::srv::AddThreeInts::Request>();
    request_2->a = msg->num;
    request_2->b = msg->num * 10;
    request_2->c = msg->num * 100;
    auto result_2 = client_2_->async_send_request(request_2, std::bind(&MinimalSubscriber::response_callback_2, this, std::placeholders::_1));
  }

  void response_callback_1(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
  {
    RCLCPP_INFO(this->get_logger(), "result1.sum: %d", future.get()->sum);
  }

  void response_callback_2(rclcpp::Client<tutorial_interfaces::srv::AddThreeInts>::SharedFuture future)
  {
    RCLCPP_INFO(this->get_logger(), "result2.sum: %d", future.get()->sum);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_1_;
  rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_2_;
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_1_;
  rclcpp::Client<tutorial_interfaces::srv::AddThreeInts>::SharedPtr client_2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
