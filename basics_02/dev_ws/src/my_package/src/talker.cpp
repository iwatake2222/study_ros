#include <cstdlib>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "my_package/talker.hpp"

#include <unistd.h>
#include <sys/types.h>
#include <sys/syscall.h>
static void print_pid_tid(const char * str)
{
  printf("[%s] pid = %ld, %d, tid = %ld\n", str, syscall(SYS_gettid), getpid(), syscall(SYS_getpid));
}

namespace my_package
{

Talker::Talker(const rclcpp::NodeOptions & options)
: Node("talker", options)
{
  print_pid_tid("Talker");
  publisher_ = this->create_publisher<std_msgs::msg::Int32>("my_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Talker::timer_callback, this));
}

void Talker::timer_callback()
{
  print_pid_tid("timer_callback");
  auto msg = std_msgs::msg::Int32();
  msg.data = count_++;
  RCLCPP_INFO(this->get_logger(), "data = %d, ptr = %p, %p", msg.data, &msg, &(msg.data));
  publisher_->publish(msg);
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_package::Talker)