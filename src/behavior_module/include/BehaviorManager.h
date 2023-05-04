#include <iostream>
#include "ros/ros.h"
#include <jsoncpp/json/json.h>
#include <string>
// #include <std_msgs/>
#include <vector>
#include <fstream>
#include <std_msgs/Header.h>
#include <behavior_module/need_msg.h>
#include <behavior_module/behavior_msg.h>
#include <behavior_module/behavior_feedback_msg.h>
#include <behavior_module/idleState.h>

#include "utils.h"
// #include <bits/stdc++.h>

using namespace std;
using namespace FABO_ROBOT;

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
    SubBehavior(){
        mActuators = vector<SubBehaviorExpression*>(5);
        mActuators[0] = new Gaze();
        mActuators[1] = new Emotion();
        mActuators[2] = new Voice();
        mActuators[3] = new Manipulator();
        mActuators[4] = new Mover();
    };
    string discription;
    vector<SubBehaviorExpression*> mActuators;
};

class Behavior{
public:
    Behavior(){
    };

    Behavior(const Behavior& beh, bool reserveStamp=true) : 
            name(beh.name), type(beh.type), current_phase(beh.current_phase), total_phase(beh.total_phase),
            target(beh.target), target_angle(beh.target_angle), target_distance(beh.target_distance),
            speech(beh.speech), rob_emotion(beh.rob_emotion), rob_emotion_intensity(beh.rob_emotion_intensity),
            weight(beh.weight), is_light(beh.is_light), necessary_count(beh.necessary_count),
            subBehaviorSeries(beh.subBehaviorSeries)
    {
        if (reserveStamp)
        {
            header.stamp = beh.header.stamp;
        }
        else{
            header.stamp = ros::Time::now();
        }
    }

    void configureByNeedMsg(const behavior_module::need_msg &msg)
    {
        name = msg.need_name;
        scene = msg.scene;
        target = msg.person_name;
        IDtype = msg.IDtype;
        target_angle = msg.target_angle;
        target_distance = msg.target_distance;
        person_emotion = msg.person_emotion;
        rob_emotion_intensity = msg.rob_emotion_intensity;
        // weight = msg.weight; // TODO：是否由需求给出权重
        speech = msg.speech;
        rob_emotion = msg.rob_emotion;
        satisfy_value = msg.satisfy_value;
        attitude = msg.attitude;
        move_speed = msg.move_speed;
        distance = msg.distance;
        voice_speed = msg.voice_speed;
    }

public:
    // params to pass to perform_module
    std_msgs::Header header;
    string name;
    string type;
    string scene;
    int current_phase = 0;
    int total_phase;
    string target;
    string IDtype;
    float target_angle;
    float target_distance;
    string speech;
    string rob_emotion;
    string person_emotion;
    int rob_emotion_intensity;
    int satisfy_value;
    string attitude;
    float move_speed;
    float distance;
    float voice_speed;
    
    // other params
    double weight;
    bool is_light;
    vector<int> necessary_count = {0,0,0,0,0};
    vector<SubBehavior> subBehaviorSeries;  // 子行为序列
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
    BehaviorManager(ros::NodeHandle& n, string data_path):n_(n)
    {
        printInColor("==================================\n", BLUE);
        printInColor(" Welcome to use behavior_module! \n", BLUE);
        printInColor("==================================\n", BLUE);
        publisher_behavior_ = n_.advertise<behavior_module::behavior_msg>("/BehaviorInstruction", 1000);
        publisher_idlestate_ = n_.advertise<behavior_module::idleState>("/idleState",1000);
        subscriber_behavior_feedback_ = n_.subscribe("/BehaviorFeedback", 1000, &BehaviorManager::behavior_feedback_callback, this);
        readinBehaviorLibrary(data_path);
        tellIdleState(true, nullptr);
    };

    /**
     * @brief Load a ROS Handle after constructing the behavior manager.
     * 
     * @param n 
     */
    void loadHandle(ros::NodeHandle& n)
    {
        n_ = n;
        publisher_behavior_ = n_.advertise<behavior_module::behavior_msg>("/BehaviorInstruction", 1000);
        subscriber_behavior_feedback_ = n_.subscribe("/BehaviorFeedback", 1000, &BehaviorManager::behavior_feedback_callback, this);
    }

    /**
     * @brief 读入行为数据库
     * 
     * @param config_file 行为数据库的json文件路径
     */
    void readinBehaviorLibrary(const string &config_file);

    Behavior* getBehaviorByName(string name);

    /**
     * @brief 读取需求话题消息
     * 
     * @param msg 接收到的需求消息 need_msg.msg
     * @return true   需求读入成功
     * @return false  需求读入失败
     */
    bool readInNewNeed(const behavior_module::need_msg &msg);

    /**
     * @brief 打印出行为数据库的数据
     * 
     */
    void printAllBehaviors();

    bool judgeSameStamp(const std_msgs::Header& header1, const std_msgs::Header& header2) {
        if (header1.stamp.sec == header2.stamp.sec)
            return true;
        else
        {
            return false;
        }
    }

    behavior_module::behavior_msg generateOrderMsgByBehavior(const Behavior& beh);
    
private:
    ros::NodeHandle n_;
    ros::Publisher publisher_behavior_;
    ros::Publisher publisher_idlestate_;
    ros::Subscriber subscriber_behavior_feedback_;

    /**
     * @brief 向行为序列加入一个新生成的行为
     * 
     * @param new_behavior 需添加的新行为
     */
    void addNewBehavior(Behavior &new_behavior);
    void tellIdleState(bool state, Behavior *completedBehavior);

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
    void behavior_feedback_callback(const behavior_module::behavior_feedback_msg &msg);
    
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

    void printBehaviors(vector<Behavior> &behaviorSeries);

    /**
     * @brief 打印出行为消息的信息
     * 
     */
    void printMsgInfo(behavior_module::behavior_msg);

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
