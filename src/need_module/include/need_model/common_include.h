/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

// define the commonly included file to avoid a long include list


// std 
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <set>
#include <unordered_map>
#include <map>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <thread>
#include <stdlib.h>
#include <cmath>
#include "yaml-cpp/yaml.h"

#include "social_msg/robot_status.h"
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include "social_msg/perception_msg.h"
#include "social_msg/need_msg.h"
#include "social_msg/robot_emotion.h"
#include "social_msg/robot_status.h"
#include "social_msg/attitude_msg.h"
#include "social_msg/attitude_query.h"
#include "social_msg/idleState.h"
#include "dynamic_reconfigure/server.h" 


// #include "perception.h"

using namespace std; 
#define NONE "\033[m"   //正常终端颜色
#define RED "\033[0;32;31m"    //红色
#define LIGHT_RED "\033[1;31m"  //粗体红色
#define GREEN "\033[0;32;32m"    //绿色
#define LIGHT_GREEN "\033[1;32m"  
#define BLUE "\033[0;32;34m"     //蓝色
#define LIGHT_BLUE "\033[1;34m"
#define DARY_GRAY "\033[1;30m"   //暗灰色
#define CYAN "\033[0;36m"
#define LIGHT_CYAN "\033[1;36m"
#define PURPLE "\033[0;35m"
#define LIGHT_PURPLE "\033[1;35m"  //淡紫色
#define YELLOW "\033[1;33m"      //黄色
#define WHITE "\033[1;37m"    //粗体白色

struct need {
    /* 信息 */
    std::string need_name = "";
    /* 评价标准 */
    std::string intention = "";
    std::string person_name = "";
    std::string IDtype = "";

    // 课题二发送的用户坐标
    double target_angle;        //交互对象角度
    double target_distance;     //交互对象距离

    std::vector<double> rob_emotion;
    std::string robot_emotion_str = "";
    int robot_emotion_intensity = 0;
    std::string person_emotion = "";
    std::vector<double> rob_status;
    /* 权重 */
    double weight = 0;
    int satisfy_value = 0;
    /* 语音内容 */
    std::string speech;
};
/* string need_name
string person_name
string IDtype
string rob_emotion
string person_emotion
float64 weight
string speech */

struct need_wu {
 
    std::string need_name;
    std::string person_name;
    std::string IDtype;
    double weight;
    double wu;
    
};


template< typename T >  
    void operator >> (const YAML::Node& node, vector<T> &v) {
        for(int i =0 ; i< node.size(); i ++)
            v.push_back(node[i].as<T>());
    }

template< typename T >  
void operator >> (const YAML::Node& node, T& str) {
   str = node.as<T>();
}

enum NeedType {
    InnerNeed = 2,
    TaskNeed = 1
};




// static int NeedTypeCheck(std::string name){
//     if(name == "Greet" || name == "MeasureTempareture" || name == "Anwser" || name == "ParentIdentity")
//         return  TaskNeed;
//     if(name == "Doubt" || name == "Chat" || name == "Charge" || name == "Wander")
//         return InnerNeed;
// }






#endif