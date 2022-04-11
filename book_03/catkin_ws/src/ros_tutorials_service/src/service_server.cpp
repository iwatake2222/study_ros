#include "ros/ros.h"
#include "ros_tutorials_service/SrvTutorial.h"

#define ADD   1
#define SUB   2
#define MUL   3
#define DIV   4

int g_operator = ADD;
 
bool calculation(ros_tutorials_service::SrvTutorial::Request& req, ros_tutorials_service::SrvTutorial::Response& res)
{
    switch (g_operator) {
    default:
    case ADD:
        res.result = req.a + req.b;
        break;
    case SUB:
        res.result = req.a - req.b;
        break;
    case MUL:
        res.result = req.a * req.b;
        break;
    case DIV:
        if (req.b == 0) {
            res.result = 0;
        } else {
            res.result = req.a / req.b;
        }
        break;
    }
    
    ROS_INFO("request: x = %ld, y = %ld", req.a, req.b);
    ROS_INFO("sending back response: %ld", res.result);
    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "service_server");
    ros::NodeHandle nh;

    nh.setParam("calculation_method", ADD);

    ros::ServiceServer server = nh.advertiseService("ros_tutorial_srv", calculation);
    ROS_INFO("ready srv server");
    ros::Rate r(10);
    while (ros::ok()) {
        nh.getParam("calculation_method", g_operator);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}