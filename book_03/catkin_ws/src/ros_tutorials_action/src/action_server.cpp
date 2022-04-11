#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "ros_tutorials_action/FibonacciAction.h"

class FibonacciAction
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ros_tutorials_action::FibonacciAction> as_;
    std::string action_name_;

    ros_tutorials_action::FibonacciFeedback feedback_;
    ros_tutorials_action::FibonacciResult result_;

public:
    FibonacciAction(std::string name):
        as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false), action_name_(name)
    {
        as_.start();
    }

    ~FibonacciAction()
    {
    }

    void executeCB(const ros_tutorials_action::FibonacciGoalConstPtr& goal)
    {
        ros::Rate r(1);
        bool success = true;

        feedback_.sequence.clear();
        feedback_.sequence.push_back(0);
        feedback_.sequence.push_back(1);

        ROS_INFO("%s: Executing, creating fibonacci sequence of order %d with seeds %i %i"
            , action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]
        );

        for (int i = 1; i <= goal->order; i++) {
           if (as_.isPreemptRequested() || !ros::ok()) {
               ROS_INFO("%s: Preempted", action_name_.c_str());
               as_.setPreempted();
               success = false;
               break;
           }

           feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i - 1]);
           as_.publishFeedback(feedback_);
           r.sleep();
        }

        if (success) {
            result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "action_server");
    FibonacciAction fibonacci("ros_tutorial_action");
    ros::spin();
    return 0;
}
