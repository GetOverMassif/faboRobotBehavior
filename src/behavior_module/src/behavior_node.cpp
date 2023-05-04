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
    ros::Subscriber subscriber_need_;
    void need_for_behavior_callback(const behavior_module::need_msg &msg);
};

BehaviorNode::BehaviorNode(ros::NodeHandle& n, string data_path)
{
    n_ = n;
    subscriber_need_ = n_.subscribe("need_lists", 1000, &BehaviorNode::need_for_behavior_callback, this);
    behaviorManager_ = new BehaviorManager(n_, data_path);
}

void BehaviorNode::need_for_behavior_callback(const behavior_module::need_msg &msg)
{
    cout << "\n---------------------------------------------------" << endl;
    printInColor("【Received need_msg】", BLUE);
    cout << msg.need_name << endl << endl;
    behaviorManager_->readInNewNeed(msg);
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior_node");
    ros::NodeHandle n;
    std::string config_file = ros::package::getPath("behavior_module") + "/data/behaviourData.json";
    // string config_file = "/home/lj/Documents/Indoor-mobile-robot-with-arms/src/behavior_module/data/behaviourData.json";
    BehaviorNode behavior_node(n, config_file);
    ros::spin();
    return 0;
}