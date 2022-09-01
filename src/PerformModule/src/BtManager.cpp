#include "BtManager.h"

namespace FABO_ROBOT
{

BtManager::BtManager(
    int &blueteeth_,
    BtRecall *btRecall):
        blueteeth(blueteeth_),
        mBtRecall(btRecall)
{
    mBtRecall->setBtManager(this);
    mptBtRecall = new thread(&FABO_ROBOT::BtRecall::run, mBtRecall);
}

void BtManager::set_ros_node(ros::NodeHandle& n){
    // on arm
    pub_robotic_arm = n.advertise<arm_control::Arms>("arm_action", 10);

    // on behavior
    sub_hehavior = n.subscribe("/BehaviorInstruction", 1000, &BtManager::behavior_callback, this);
    pub_hehaviorFeedback = n.advertise<BehaviorModule::behavior_feedback_msg>("/BehaviorFeedback", 1000);
}

void BtManager::sendBtData(string str){
    int length = str.size();
    char *temp = (char*)str.c_str();
    char str_char[length + 2];
    strcpy(str_char, temp);

    // print out the data
    printInColor("正在发送蓝牙数据 : ", BLUE);
    printInColor(str_char, BLUE);
    printf("\n");

    write(blueteeth, str_char, strlen(str_char));

    printInColor("发送完成\n\n", BLUE);
}

void BtManager::processBtData(string btData_str){
    stringstream ss(btData_str);
    
    //处理数据
    string type;
    ss >> type;  // 读取接收到的蓝牙数据的第一个字符串 指令类型
    if (type == "BehaviorFeedback") {
        cout << "type = " << type << endl;
        processBehaviorFeedback(btData_str);
    }
    else if (type == "ArmControl"){
        cout << "type = " << type << endl;
        processArmControl(btData_str);
    }
    else {
        printInColor(type.c_str(), RED);
        printInColor(" 不是正确的指令类型\n", RED);
    }
}

/**
 * @brief 通过蓝牙发送指令，并且发布对机械臂的控制
 * 
 * @param parameter_blueteeth 需要通过蓝牙发送的字符串（string）
 * @param arm_action 调用机械臂的动作（string）
 * @param arm_rate 
 */
void BtManager::sendBtMsgAndControlArm(string parameter_blueteeth, string arm_action, int arm_rate)
{
    char *temp = (char*)parameter_blueteeth.c_str();
    char parameter_blueteeth_char[1000];
    strcpy(parameter_blueteeth_char, temp);
    printf("行为参数 parameter_blueteeth_char : %s \n", parameter_blueteeth_char);
    write(blueteeth, parameter_blueteeth_char, strlen(parameter_blueteeth_char));

    if (arm_action != "none") {
        arm_control::Arms arm_msg;
        arm_msg.action = arm_action;
        arm_msg.rate = arm_rate;
        pub_robotic_arm.publish(arm_msg);
    }
}

// private

void BtManager::behavior_callback(const BehaviorModule::behavior_msg &msg){
    std::string bluetooth_message = "";
    bluetooth_message = bluetooth_message
                        + msg.name + "," 
                        + to_string(msg.current_phase) + "/" + to_string(msg.total_phase);
    sendBtData(bluetooth_message);
    return;
}

void BtManager::processBehaviorFeedback(string behaviorFeedback_str){
    BehaviorModule::behavior_feedback_msg msg;
    stringstream ss(behaviorFeedback_str);
    string type, phase_str;
    ss >> type;
    if(type != "BehaviorFeedback"){
        printf("Error when process BehaviorFeedback\n");
        return;
    }
    ss >> msg.hehavior_name >> phase_str;
    msg.current_phase = atoi(phase_str.c_str());
    // cout << "pub前" << msg.hehavior_name << " " << msg.current_phase << endl;

    unique_lock<mutex> lock(mutexPub);
    pub_hehaviorFeedback.publish(msg);
    printInColor("发出行为反馈话题 : ", GREEN);
    // cout << "pub后" << msg.hehavior_name << " " << msg.current_phase << endl;
}

void BtManager::processArmControl(string armControl_str){
    arm_control::Arms msg;
    stringstream ss(armControl_str);
    string type, phase_str;
    ss >> type;
    if(type != "ArmControl"){
        printf("Error when process ArmsControl\n");
        return;
    }
    ss >> msg.action;
    unique_lock<mutex> lock(mutexPub);
    pub_robotic_arm.publish(msg);
    printInColor("发出机械臂动作话题 : ", GREEN);
    cout << msg.action << endl;
}

};
