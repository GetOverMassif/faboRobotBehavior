/*
 * @Author: GetOverMassif 164567487@qq.com
 * @Date: 2022-08-06 18:17:49
 * @LastEditors: GetOverMassif 164567487@qq.com
 * @LastEditTime: 2022-09-01 19:50:16
 * @FilePath: /Indoor-mobile-robot-with-arms/src/arm/arm_control/src/arm_control.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
//
// Created by lj on 2022/3/18.
//

#include "string"
#include <iostream>
#include <vector>
#include "ros/ros.h"

#include "std_msgs/String.h"

#include "ArmsActionManager.h"
#include "ArmsControl.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_control");
    ros::NodeHandle n;
    string config_file = "/home/lj/Documents/Indoor-mobile-robot-with-arms/src/arm/arm_control/json/arms_actions.json";
    ArmsActionManager arms_action_manager(config_file);
    
    // 创建机械臂控制实例
    ArmsControl arm_control(n);
    // 加入动作
    arm_control.readinArmActions(config_file);
    
    sleep(2);

    arm_control::Arms msg;
    string action_str = "sleep";
    msg.action = action_str;
    arm_control.callAction(msg);

    ros::spin();

    return 0;
}