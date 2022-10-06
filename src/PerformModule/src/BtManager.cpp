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

    // wheel/head motion
    sub_wheelMotion = n.subscribe("/wheelMotion", 1000, &BtManager::wheelMotion_callback, this);
    sub_headMotion = n.subscribe("/headMotion", 1000, &BtManager::headMotion_callback, this);

    // /turtle1/cmd_vel
    sub_cmdVel = n.subscribe("/turtle1/cmd_vel", 1, &BtManager::cmdVel_callback, this);
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
void BtManager::wheelMotion(const int v_left, const int v_right, const int time) {
    string order = "Wheel," + std::to_string(v_left) +
                        "," + std::to_string(v_right) +
                        "," + std::to_string(time);
    cout << "order = " << order << endl;
    sendBtData(order);
}

void BtManager::headMotion(const int angle, const int vel) {
    string order = "Head," + std::to_string(angle) +
                        "," + std::to_string(vel);
    cout << "order = " << order << endl;
    sendBtData(order);
}

void BtManager::behavior_callback(const BehaviorModule::behavior_msg &msg){
    std::string bluetooth_message = "";
    bluetooth_message = bluetooth_message
                        + msg.name + "," 
                        + to_string(msg.current_phase) + "/" + to_string(msg.total_phase);
    sendBtData(bluetooth_message);
    return;
}

void BtManager::wheelMotion_callback(const PerformModule::WheelMotion_msg &msg) {
    // v_left, v_right(-500~500 mm/s), time(ms)
    int v_left = msg.v_left, v_right = msg.v_right, time = msg.time;
    wheelMotion(v_left, v_right, time);
}

void BtManager::headMotion_callback(const PerformModule::HeadMotion_msg &msg) {
    // angle(0-240 degree), a_velocity(deg/s)
    int angle = msg.angle, vel = msg.vel;
    headMotion(angle, vel);
}

void BtManager::cmdVel_callback(const geometry_msgs::Twist msg) {
    // normal : vl = 2/-2, z = 2/-2, D = 500, kl = 0.01, ka = 1.5
    // vl = kl * (v_left + v_right) / 2, va = ka * (v_right - v_left) / D
    float D = 500.0, kl = 0.01, ka = 2.0;
    float vl = msg.linear.x, va = msg.angular.z;
    float v_sum = (vl * 2.0 / kl), v_dif = va * D / ka;
    int v_left = (int)(v_sum - v_dif) / 2;
    int v_right = (int)(v_sum + v_dif) / 2;
    wheelMotion(v_left, v_right, 600);
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

void BtManager::processArmControl(string armControl_str) {
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

} // end of namespace FABOROBOT