/*
 * @Author: GetOverMassif 164567487@qq.com
 * @Date: 2022-08-06 18:17:49
 * @LastEditors: GetOverMassif 164567487@qq.com
 * @LastEditTime: 2022-09-01 15:40:33
 * @FilePath: /Indoor-mobile-robot-with-arms/src/arm/arm_control/include/ArmsActionManager.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE#
 */

#ifndef ARMACTIONMANAGER_H
#define ARMACTIONMANAGER_H

#include <ros/ros.h>
#include <jsoncpp/json/json.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <bits/stdc++.h>
#include <utils.h>

using namespace std;
using namespace FABO_ROBOT;

/*
* 定义层次：
    机械臂动作管理器{
        机械臂动作名称目录
        机械臂动作库
            机械臂动作
                机械臂配置序列
                    机械臂配置
    }
*/

class ArmConfig{
public:
    
    ArmConfig(){}

    ArmConfig(bool keep_still,double *joint_position,double move_speed,int pauseTime){
        keepStill = keep_still;
        for(int i=0 ; i<6 ; i++)
            jointPos[i] = *(joint_position+i);
        speed = move_speed;
        mPauseTime = pauseTime;
    }

    const ArmConfig& operator=(const ArmConfig& config){
        keepStill = config.keepStill;
        for (int i = 0 ; i < 6 ; i++){
            jointPos[i] = config.jointPos[i];
        }
        speed = config.speed;
        mPauseTime = config.mPauseTime;
        return *this;
    }

    bool keepStill;
    double jointPos[6];
    double speed;
    int mPauseTime;  // ms
};

class ArmsAction{
public:
    ArmsAction(string name_,vector<ArmConfig> left_arm_action_,vector<ArmConfig> right_arm_action_){
        name = name_;
        left_arm_action = left_arm_action_;
        right_arm_action = right_arm_action_;
    }
    string name;
    vector<ArmConfig> left_arm_action;
    vector<ArmConfig> right_arm_action;
};

class ArmsActionManager{
public:
    ArmsActionManager(const string &config_file){
        readinActions(config_file);
        // printAllActions();
    }
    
    void printAllActions();
    map<std::string, ArmsAction> get_arms_actions(){
        return action_library;
    };

private:
    void readinActions(const string &config_file);
    void readinArmConfig(Json::Value root,std::vector<ArmConfig>& single_arm_action);
    set<std::string> action_catalog;
    map<std::string, ArmsAction> action_library;
};

#endif