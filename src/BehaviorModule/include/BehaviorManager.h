#include <iostream>
#include "ros/ros.h"
#include <jsoncpp/json/json.h>
#include <string>
// #include <std_msgs/>
#include <vector>
#include <fstream>
#include <BehaviorModule/need_msg.h>
#include <BehaviorModule/behavior_msg.h>
#include <BehaviorModule/behavior_feedback_msg.h>
// #include <bits/stdc++.h>

using namespace std;

struct need{
    string name;
    string type;
    string object;
    string IDtype;
};

enum needtype{
    EXPRESSION_NEED = 0,
    INTERATION_NEED = 1,
    OPERATION_NEED = 2
};

// 子行为表达
class SubBehaviorExpression{
public:
    SubBehaviorExpression(){};
    bool is_necessary;  // 动作对行为是否必要
    double weight;   // 动作权重
};

// 视线
class Gaze : public SubBehaviorExpression{
public:
    string target;  // 目标对象
    double rate;  // 速率
};

// 情绪
class Emotion : public SubBehaviorExpression{
public:
    string type;  // 情绪类型
};

// 语音
class Voice : public SubBehaviorExpression{
public:
    string content;  // 语音内容
    int tone;// 语调（低沉、一般、高昂）
    double rate;  // 速率
};

// 操作
class Manipulator : public SubBehaviorExpression{
public:
    string action;  // 动作
    string target;  // 对象
    double rate;  // 速率
    double range; // 幅度
};

// 移动
class Mover : public SubBehaviorExpression{
public:
    string target;  // 目标（object/around）
    double rate;  // 速率
    double distance;  // 距离
    double range;  // 范围
};

// 子行为
class SubBehavior{
public:
    SubBehavior(){};
    string discription;
    Gaze gaze;
    Emotion emotion;
    Voice voice;
    Manipulator manipulator;
    Mover mover;
};

class Behavior{
public:
    Behavior(){};

public:
    string name;
    double weight;
    string type;
    bool is_light;
    string target;
    int behavior_phase = 0;
    vector<SubBehavior> subBehaviorSeries;  // 子行为序列
    vector<int> necessary_count = {0,0,0,0,0};
};

// 行为管理
class BehaviorManager{
public:
    BehaviorManager(ros::NodeHandle& n){
        n_ = n;
        publisher_behavior_ = n_.advertise<BehaviorModule::behavior_msg>("/BehaviorInstruction", 1000);
        subscriber_behavior_feedback_ = n_.subscribe("/BehaviorFeedback", 1000, &BehaviorManager::behavior_feedback_callback, this);
    };
    void readinBehaviorLibrary(const string &config_file);
    bool readInNewNeed(const BehaviorModule::need_msg &msg);
    void updateBehaviorPub();
    void printAllBehaviors();
    
private:
    ros::NodeHandle n_;
    ros::Publisher publisher_behavior_;
    ros::Subscriber subscriber_behavior_feedback_;

    void addNewBehavior(Behavior new_behavior);
    void behavior_feedback_callback(const BehaviorModule::behavior_feedback_msg &msg);
    int insertBehavior(Behavior &new_behavior);
    void printCurrentSeries();
    void printMsgInfo(BehaviorModule::behavior_msg);
    set<std::string> behavior_catalog;
    map<string,Behavior> behavior_library;
    vector<Behavior> behaviorSeries;
    vector<Behavior> parallelBehaviorSeries;
    vector<int> occupancy = {1,1,1,1,1};
    int computeParallel();
    bool behaviorChangeFlag = false;
    int parallelNum = 1;
};