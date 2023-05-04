/*
 * @Descripttion: 
 * @version: 
 * @Author: Zhang Jiadong
 * @Date: 2021-12-29 11:42:24
 * @LastEditors: GetOverMassif 164567487@qq.com
 * @LastEditTime: 2022-08-27 22:49:32
 */

#ifndef BTRECALL_H
#define BTRECALL_H

#include "utils.h"
#include "BtManager.h"
#include <mutex>

using namespace std;
using namespace FABO_ROBOT;

namespace FABO_ROBOT
{

class BtManager;

class BtRecall
{
public:
    static BtRecall *m_g_recall;

    BtRecall(/* args */){};

    BtRecall(int* blueteeth_);
    
    ~BtRecall(){}

    void setBtManager(BtManager *btmanager);

    void run();

private:
    int blueteeth;
    BtManager *mBtManager;
    string mbluetoothData;
    bool flag_gaze_bt = 0; //, flag_screen_bt=0, flag_sounder_bt=0, flag_arm_bt=0, flag_leg_bt=0;
    int ret;
    pthread_t th;

    static bool newDataReceived;
    static string bluetoothData;
    static std::mutex *mMutexBtData;

    bool checkBtNewData();

    static void recordBtData(string btData_str);
    
    static void *pthread_read(void *blueteeth);
};

}

#endif