#include <iostream>
#include <vector>

using namespace std;

// 子行为表达
class SubBehaviorExpression{
public:
    bool is_necessary;  // 动作对行为是否必要
    double weight;   // 动作权重
    double startTime;  // 开始时间
    double endtime;  // 结束时间
    bool is_visible;  // 是否可视化
}

// 视线
class Gaze : public Action{
public:
    string target;  // 目标对象
    double rate;  // 速率
}

// 情绪
class Emotion : public Action{
public:
    string type;  // 情绪类型
}

// 语音
class Voice : public Action{
public:
    string content;  // 语音内容
    int tone；// 语调（低沉、一般、高昂）
    double rate;  // 速率
}

// 操作
class Manipulator : public Action{
public:
    string action;  // 动作
    string target;  // 对象
    double rate;  // 速率
    double range; // 幅度
}

// 移动
class Mover : public Action{
public:
    string target;  // 目标（object/around）
    double rate;  // 速率
    double distance;  // 距离
    double range;  // 范围
}

// 子行为
class SubBehavior{
public:
    string name;
    Gaze gaze;
    Emotion emotion;
    Voice voice;
    Arms arms;
    Legs legs;
}

class Behavior{
public:
    vector<SubBehavior> subBehaviorSeries;  // 子行为序列
    bool interruptType;  // 行为打断类型（紧急打断/非紧急打断）
    int execute_flag;  // 执行进度标志
    vector<bool> exist_f_transition;  //是否存在过渡行为
    vector<bool> exist_b_transition;  //是否存在过渡行为
    vector<SubBehavior*> ;  // 前置过渡行为；
    vector<SubBehavior*> ;  // 后置过渡行为；
}

// 行为管理
class BehaviorManager{
public:
    vector<Behavior> behaviorSeries;
    void readinBehaviorLibrary(string &config_file);
    
    void addBehavior(Behavior){
        int order = checkOrder();  // 存储新加入行为权重顺序
        if(order>2){  // 如果权重不在前2
            // 将行为插入行为序列order位置
        }
        else if (checkConcurrence()){  // 否则，检查能够并发，如能并发
            // 融合行为
        }
        else{
            // 打断插入
        }
    };
    
private:
    map<string,Behavior> behavior_library;
    vector<Behavior> behavior_list;
}