/*
 * @Descripttion: 
 * @version: 
 * @Author: Zhang Jiadong
 * @Date: 2022-08-24 20:58:00
 * @LastEditors: GetOverMassif 164567487@qq.com
 * @LastEditTime: 2022-10-06 13:36:50
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
    
    // int op;
    // cout << "\n1-wheel,2-head,0-exit" << endl << "op:";
    // cin >> op;
    
    // while (op){
    //     if (op == 1) {
    //         int v_left, v_right, time;
    //         cout << "v_left, v_right(-500~500 mm/s), time(ms) : ";
    //         cin >> v_left >> v_right >> time;

    //         string order = "Wheel," + std::to_string(v_left) +
    //                             "," + std::to_string(v_right) +
    //                             "," + std::to_string(time);
    //         cout << "order = " << order << endl;
    //         btmanager.sendBtData(order);
    //     }
    //     else if (op == 2) {
    //         int angle, vel;
    //         cout << "angle(0-240 degree), a_velocity(deg/s) : ";
    //         cin >> angle >> vel;
    //         string order = "Head," + std::to_string(angle) +
    //                             "," + std::to_string(vel);
    //         cout << "order = " << order << endl;
    //         btmanager.sendBtData(order);
    //     }
    //     cout << "\n1-wheel,2-head,0-exit" << endl << "op:";
    //     cin >> op;
    // }

    return 0;
}
