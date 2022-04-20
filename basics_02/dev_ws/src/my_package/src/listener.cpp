#include <cstdlib>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "my_package/listener.hpp"

#include <unistd.h>
#include <sys/types.h>
#include <sys/syscall.h>
static void print_pid_tid(const char * str)
{
  printf("[%s] pid = %ld, %d, tid = %ld\n", str, syscall(SYS_gettid), getpid(), syscall(SYS_getpid));
}

namespace my_package
{

Listener::Listener(const rclcpp::NodeOptions & options)
: Node("listener", options)
{
  print_pid_tid("Listener");
  subscription_1_ = this->create_subscription<std_msgs::msg::Int32>("my_topic", 10, std::bind(&Listener::topic_callback, this, std::placeholders::_1));
}

void Listener::topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  print_pid_tid("topic_callback");
  RCLCPP_INFO(this->get_logger(), "data = %d, ptr = %p", msg->data, msg.get());
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_package::Listener)