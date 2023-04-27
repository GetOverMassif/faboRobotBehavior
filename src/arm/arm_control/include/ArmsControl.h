/*
 * @Author: GetOverMassif 164567487@qq.com
 * @Date: 2022-08-31 22:12:37
 * @LastEditors: GetOverMassif 164567487@qq.com
 * @LastEditTime: 2022-09-01 19:47:35
 * @FilePath: /Indoor-mobile-robot-with-arms/src/arm/arm_control/include/ArmsControl.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef ARMSCONTROL_H
#define ARMSCONTROL_H

#include "string"
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "ArmsActionManager.h"
#include "arm_control/Hand_Control.h"
#include <rm_msgs/Plan_State.h>
#include <rm_msgs/Tool_Analog_Output.h>
#include <rm_msgs/Tool_Digital_Output.h>
#include <rm_msgs/MoveJ.h>
#include <arm_control/Arms.h>
#include "utils.h"

#include <mutex>
#include <thread>

#include "ArmsActionManager.h"
#include "ArmController.h"

using namespace std;

class ArmsActionManager;
class ArmController;

class ArmsControl
{
public:
    ArmsControl(ros::NodeHandle& n);
    void readinArmActions(const string &config_file);
    void callAction(const arm_control::Arms &msg);

private:
    ros::NodeHandle n_;
    ros::Subscriber subscriber_arm_;
    ros::Subscriber subscriber_arm1_arrival_;
    ros::Subscriber subscriber_arm2_arrival_;
    ros::ServiceServer service_hold_hand_;
    ros::ServiceServer service_open_hand_;
    ros::Publisher d_output_pub_;
    ros::Publisher a_output_pub_;
    ArmsActionManager *action_manager;
    ArmController *mLeftArm;
    ArmController *mRightArm;

    std::mutex mutexArmMoveJ[2];
    bool PlanState[2] = {true, true};
    rm_msgs::MoveJ moveJ_msg;

    void arm_action_callback(const arm_control::Arms &msg);
    bool hand_callback(arm_control::Hand_Control::Request &req,
                            arm_control::Hand_Control::Response &res);
};

#endif