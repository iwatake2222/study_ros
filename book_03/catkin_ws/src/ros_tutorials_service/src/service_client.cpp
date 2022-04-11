#include <cstdlib>
#include "ros/ros.h"
#include "ros_tutorials_service/SrvTutorial.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "service_client");
    if (argc != 3) {
        ROS_ERROR("cmd: rosrun ros_tutorials_service service_clieng arg0 arg1");
        return 1;
    }

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<ros_tutorials_service::SrvTutorial>("ros_tutorial_srv");

    ros_tutorials_service::SrvTutorial srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);

    if (client.call(srv)) {
        ROS_INFO("send srv, %ld %ld", srv.request.a, srv.request.b);
        ROS_INFO("recv srv, %ld", srv.response.result);
    } else {
        ROS_ERROR("Failed to call ros_tutotial_srv");
        return 1;
    }

    return 0;
}