#include <cstdlib>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "my_package/listener.hpp"
#include "my_package/talker.hpp"

#include <unistd.h>
#include <sys/types.h>
#include <sys/syscall.h>
static void print_pid_tid(const char * str)
{
  printf("[%s] pid = %ld, %d, tid = %ld\n", str, syscall(SYS_getpid), getpid(), syscall(SYS_gettid));
}

int main(int argc, char * argv[])
{
  print_pid_tid("main");
  
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
//   rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto talker = std::make_shared<my_package::Talker>(options);
  exec.add_node(talker);
  auto listener = std::make_shared<my_package::Listener>(options);
  exec.add_node(listener);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
