/*
 * @Descripttion: 
 * @version: 
 * @Author: Zhang Jiadong
 * @Date: 2021-12-29 11:40:12
 * @LastEditors: GetOverMassif 164567487@qq.com
 * @LastEditTime: 2022-10-06 15:29:13
 */

#ifndef BTMANAGER_H
#define BTMANAGER_H

#include <BehaviorModule/behavior_msg.h>
#include <PerformModule/HeadMotion_msg.h>
#include <PerformModule/WheelMotion_msg.h>
#include <geometry_msgs/Twist.h>
#include <BehaviorModule/behavior_feedback_msg.h>
#include "utils.h"
#include <string>
#include "BtRecall.h"
#include "ros/ros.h"
#include <thread>
#include <mutex>

using namespace std;

class BtRecall;

namespace FABO_ROBOT
{

class BtManager{
public:
    BtManager(int &blueteeth_, BtRecall *btRecall);

    ~BtManager(){}

    void set_ros_node(ros::NodeHandle& n);
    
    void sendBtData(string str);

    void processBtData(string btData_str);

    /**
     * @brief 通过蓝牙发送指令，并且发布对机械臂的控制
     * 
     * @param parameter_blueteeth 需要通过蓝牙发送的字符串（string）
     * @param arm_action 调用机械臂的动作（string）
     * @param arm_rate 
     */
    void sendBtMsgAndControlArm(string parameter_blueteeth, string arm_action, int arm_rate);

private:
    int blueteeth;
    std::mutex mutexPub;
    BtRecall *mBtRecall;
    std::thread *mptBtRecall;
    string gaze_target_lastpub = "none";
    string screen_type_lastpub = "none";
    ros::Publisher pub_robotic_arm;  //机械臂通信专用节点
    
    ros::Publisher pub_hehaviorFeedback;
    ros::Subscriber sub_hehavior;
    ros::Subscriber sub_wheelMotion;
    ros::Subscriber sub_headMotion;
    ros::Subscriber sub_cmdVel;
    
    void wheelMotion(const int v_left, const int v_right, const int time);
    void headMotion(const int angle, const int vel);

    void behavior_callback(const BehaviorModule::behavior_msg &msg);
    void wheelMotion_callback(const PerformModule::WheelMotion_msg &msg);
    void headMotion_callback(const PerformModule::HeadMotion_msg &msg);
    void cmdVel_callback(const geometry_msgs::Twist msg);

    void processBehaviorFeedback(string behaviorFeedback_str);
    void processArmControl(string armControl_str);
};

}

#endif
