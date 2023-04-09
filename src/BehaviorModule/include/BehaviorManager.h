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
    int current_phase = 0;
    int total_phase;
    vector<SubBehavior> subBehaviorSeries;  // 子行为序列
    vector<int> necessary_count = {0,0,0,0,0};
};

/**
 * @brief 行为管理器
 * 
 */
class BehaviorManager
{
public:

    /**
     * @brief Construct a new Behavior Manager object without parameter;
     * 
     */
    BehaviorManager();
    
    /**
     * @brief Construct a new Behavior Manager object
     * 
     * @param n 行为管理器所关联的节点句柄，用于接受和发送话题。
     */
    BehaviorManager(ros::NodeHandle& n):n_(n)
    {
        publisher_behavior_ = n_.advertise<BehaviorModule::behavior_msg>("/BehaviorInstruction", 1000);
        subscriber_behavior_feedback_ = n_.subscribe("/BehaviorFeedback", 1000, &BehaviorManager::behavior_feedback_callback, this);
    };

    /**
     * @brief Load a ROS Handle after constructing the behavior manager.
     * 
     * @param n 
     */
    void loadHandle(ros::NodeHandle& n)
    {
        n_ = n;
        publisher_behavior_ = n_.advertise<BehaviorModule::behavior_msg>("/BehaviorInstruction", 1000);
        subscriber_behavior_feedback_ = n_.subscribe("/BehaviorFeedback", 1000, &BehaviorManager::behavior_feedback_callback, this);
    }

    /**
     * @brief 读入行为数据库
     * 
     * @param config_file 行为数据库的json文件路径
     */
    void readinBehaviorLibrary(const string &config_file);

    /**
     * @brief 读取需求话题消息
     * 
     * @param msg 接收到的需求消息 need_msg.msg
     * @return true   需求读入成功
     * @return false  需求读入失败
     */
    bool readInNewNeed(const BehaviorModule::need_msg &msg);

    /**
     * @brief 打印出行为数据库的数据
     * 
     */
    void printAllBehaviors();
    
private:
    ros::NodeHandle n_;
    ros::Publisher publisher_behavior_;
    ros::Subscriber subscriber_behavior_feedback_;

    /**
     * @brief 向行为序列加入一个新生成的行为
     * 
     * @param new_behavior 需添加的新行为
     */
    void addNewBehavior(Behavior new_behavior);

    /**
     * @brief 更新行为消息的发布
     * 
     */
    void updateBehaviorPub();

    /**
     * @brief 对执行模块反馈的子行为执行消息做出反应
     * 
     * @param msg 行为执行情况的反馈话题 behavior_feedback_msg.msg
     */
    void behavior_feedback_callback(const BehaviorModule::behavior_feedback_msg &msg);
    
    /**
     * @brief 向总的行为序列中插入一个行为
     * 
     * @param new_behavior 需要插入的新行为
     * @return int 行为插入的位置
     */
    int insertBehavior(Behavior &new_behavior);

    /**
     * @brief 计算当前总的行为序列可并行行为的数量
     * 
     * @return int 可并行行为的数量
     */
    int computeParallel();

    /**
     * @brief 打印出当前总行为序列的信息
     * 
     */
    void printCurrentSeries();

    /**
     * @brief 打印出行为消息的信息
     * 
     */
    void printMsgInfo(BehaviorModule::behavior_msg);

    // 数据库所有行为名称的集合，用于在响应需求时判断是否有对应的行为
    set<std::string> behavior_catalog;

    // 行为数据库
    map<string,Behavior> behavior_library;

    // 需要执行的总的行为序列，按权重从大到小排列
    vector<Behavior> behaviorSeries;

    // 当前正在并行执行的行为序列
    vector<Behavior> mvCurrentBehaviors;

    // 存放占用各动作表现形式的行为序号
    vector<int> occupancy = {1,1,1,1,1};

    // 标记当前是否处于等待停顿状态
    bool pauseFlag = false;

    // 存储最新算得的可并行行为数量
    int parallelNum = 1;
};
