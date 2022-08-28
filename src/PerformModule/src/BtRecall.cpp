/*
 * @Author: GetOverMassif 164567487@qq.com
 * @Date: 2022-08-27 21:33:06
 * @LastEditors: GetOverMassif 164567487@qq.com
 * @LastEditTime: 2022-08-28 16:31:24
 * @FilePath: /Indoor-mobile-robot-with-arms/src/PerformModule/src/BtRecall.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "BtRecall.h"

namespace FABO_ROBOT
{

bool BtRecall::newDataReceived = false;
string BtRecall::bluetoothData = "";
std::mutex *BtRecall::mMutexBtData = new std::mutex();

BtRecall::BtRecall(int* blueteeth_):blueteeth(*blueteeth_){
    m_g_recall = this; 
    pthread_create(&th, NULL, pthread_read, (void *)blueteeth_);
}

void BtRecall::setBtManager(BtManager *btmanager){
    mBtManager = btmanager;
}

void BtRecall::run() {
    while (1) {
        static int i = 1;
        if(i){
            printInColor("已经进入recall的循环程序\n", BLUE);
            i--;
        }
        if (checkBtNewData()) {
            printf("BtRecall::checkBtNewData, 正在传给mBtManager\n");
            mBtManager->processBtData(mbluetoothData);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
}

bool BtRecall::checkBtNewData(){
    unique_lock<mutex> lock(*mMutexBtData);
    if(newDataReceived){
        newDataReceived = false;
        mbluetoothData = bluetoothData;
        return true;
    }
    else{
        return false;
    }
}

void BtRecall::recordBtData(string btData_str){
    unique_lock<mutex> lock(*mMutexBtData);
    bluetoothData = btData_str;
    newDataReceived = true;
}

void *BtRecall::pthread_read(void *blueteeth)
{
    int fd, ret;
    fd = *(int *)(blueteeth);
    while (1)
    {   //循环读取串口数据，读到buf
        char btData[128] = {'\0'};
        ret = read(fd, btData, sizeof btData);
        if (ret > 0){
            printInColor("\n收到蓝牙数据 : ", FABO_ROBOT::PINKISH_RED);
            printInColor(btData, FABO_ROBOT::PINKISH_RED);
            printf("\n");
            
            string btData_str = btData;
            recordBtData(btData_str);
            fflush(stdout);
        }
        else if (ret < 0) {
            printInColor("read error ret : ", FABO_ROBOT::PINKISH_RED);
            printf("ret = %d\n", ret);
            return NULL;
        }
    }
}

};
