/*
 * @Descripttion: 
 * @version: 
 * @Author: Zhang Jiadong
 * @Date: 2022-08-24 20:58:00
 * @LastEditors: GetOverMassif 164567487@qq.com
 * @LastEditTime: 2022-08-28 00:18:29
 */

//ros头文件
#include <ros/ros.h>
#include "std_msgs/String.h"

#include <thread>
#include "common_include.h"
#include "BtManager.h"
// #include "BtRecall.h"
// 蓝牙通信  TODO: 无线通讯通道
#include "serial.h"

using namespace std;
using namespace FABO_ROBOT;

// ros node
ros::Subscriber sub_behavior;
ros::Publisher pub_reply;
ros::Publisher pub_need_satisfy;
ros::Publisher pub_associated_need;
ros::Publisher pub_body_status;

// 蓝牙
#define PORT "/dev/rfcomm0"
// #define PORT "/dev/ttyUSB0"
#define BAUDRATE 115200
int ret;
pthread_t th;

//创建接收线程，用于读取串口数据
char port[] = {PORT};
int bluetooth = open_serial(port, BAUDRATE, 8, 'N', 1);

/******************************************************************
 * 一、BtRecall函数：  btRecall会不断的接收机器人发送的信息，并存储在btRecall中的成员变量中。
*******************************************************************/
// BtRecall* BtRecall::m_g_recall;  // 初始化BtRecall类中的静态变量

FABO_ROBOT::BtRecall* FABO_ROBOT::BtRecall::m_g_recall;
// FABO_ROBOT::BtRecall::newDataReceived = false;

int main(int argc, char** argv){

    // FABO_ROBOT::initBtRecallStaticMember();
    

    FABO_ROBOT::BtRecall *btRecall = new FABO_ROBOT::BtRecall(&bluetooth);

    FABO_ROBOT::BtManager btmanager(bluetooth, btRecall);
    // ROS node
    ros::init(argc, argv, "blueteeth_demo_node");
    ros::NodeHandle n;

    printInColor("已启动动作执行模块\n\n", GREEN);
    
    // pub中会用到ros节点，以给机械臂发送topic
    btmanager.set_ros_node(n);

    ros::spin();

    // string parameter_blueteeth = "ParentsValidation,0/2";
    // string arm_action = "wave";
    // int arm_rate = 2 ;
    
    // int op = 0;
    // while (op != 3){
    //     cout << "behavior_msg:";
    //     cin >> parameter_blueteeth;
    //     btmanager.sendBtMsgAndControlArm(parameter_blueteeth,  arm_action  , arm_rate);
    //     cout << "\n0-continue,1-pause,3-exit" << endl << "op:";
    //     cin >> op;
    //     if(op == 1){
    //         parameter_blueteeth = "pause,1/2";
    //         btmanager.sendBtMsgAndControlArm(parameter_blueteeth,  arm_action  , arm_rate);
    //     }
    // }

    return 0;
}
