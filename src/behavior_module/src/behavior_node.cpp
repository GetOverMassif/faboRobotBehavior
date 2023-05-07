//
// Created by lj on 2022/6/28.
//

#include "string"
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include <ros/package.h>

#include "std_msgs/String.h"

#include "BehaviorManager.h"
#include <behavior_module/need_msg.h>

class BehaviorNode
{
public:
    BehaviorNode(ros::NodeHandle& n, string data_path);

private:
    ros::NodeHandle n_;
    BehaviorManager *behaviorManager_;
};

BehaviorNode::BehaviorNode(ros::NodeHandle& n, string data_path)
{
    n_ = n;
    behaviorManager_ = new BehaviorManager(n_, data_path);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior_node");
    ros::NodeHandle n;
    std::string config_file = ros::package::getPath("behavior_module") + "/data/behaviourData.json";
    BehaviorNode behavior_node(n, config_file);
    ros::spin();
    return 0;
}