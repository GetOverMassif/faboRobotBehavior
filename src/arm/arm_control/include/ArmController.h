/*
 * @Author: GetOverMassif 164567487@qq.com
 * @Date: 2022-09-01 10:21:19
 * @LastEditors: GetOverMassif 164567487@qq.com
 * @LastEditTime: 2022-09-01 19:49:34
 * @FilePath: /Indoor-mobile-robot-with-arms/src/arm/arm_control/include/ArmController.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%A
 */
#ifndef ARMCONTROLLER_H
#define ARMCONTROLLER_H

#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#include <thread>

#include "ArmsActionManager.h"
#include "common_include.h"

#include <ros/ros.h>
#include <rm_msgs/MoveJ.h>
#include <rm_msgs/Tool_Digital_Output.h>
#include <rm_msgs/Tool_Analog_Output.h>
#include <rm_msgs/Plan_State.h>


using namespace std;

class ArmController
{
public:
    ArmController(ros::NodeHandle& rosNode, int ArmNumber);
    void updateMoveJ(std::vector<ArmConfig> armConfigs);
    void updateHand(bool handState);

private:
    void pubMoveJ();
    void pubHand();
    void PlanStateCallback(const rm_msgs::Plan_State &msg);
    void printNum();

    int mArmNumber;
    ros::NodeHandle mRosNode;
    ros::Publisher mPubMoveJ;
    ros::Publisher mPubToolDOutput;
    ros::Publisher mPubToolAOutput;
    ros::Subscriber mSubPlanState;

    mutex mutexArmConfigs;
    vector<ArmConfig> mArmConfigs;
    thread *mtpubMoveJ;
    
    mutex mutexHand;
    bool mHandState;  // true: open, false
    thread *mtpubHand;
};

#endif