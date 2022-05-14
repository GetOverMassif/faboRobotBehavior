#include <ros/ros.h>
#include <jsoncpp/json/json.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <bits/stdc++.h>

using namespace std;

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
    ArmConfig(bool keep_still,double *joint_position,double move_speed,double pauseTime){
        keepStill = keep_still;
        for(int i=0 ; i<6 ; i++)
            jointPos[i] = *(joint_position+i);
        speed = move_speed;
        rate = new ros::Rate(1.0/pauseTime);
    }
    bool keepStill;
    double jointPos[6];
    double speed;
    ros::Rate *rate;
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
        printAllActions();
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
