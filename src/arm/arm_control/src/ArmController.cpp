/*
 *@Author: GetOverMassif 164567487@qq.com
 * @Date: 2022-09-01 10:30:35
 * @LastEditors: GetOverMassif 164567487@qq.com
 * @LastEditTime: 2022-09-01 19:57:02
 * @FilePath: /Indoor-mobile-robot-with-arms/src/arm/arm_control/src/ArmController.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AEc
 */

#include "ArmController.h"

ArmController::ArmController(ros::NodeHandle& rosNode, int ArmNumber)
      :mRosNode(rosNode),mArmNumber(ArmNumber){
    mRosNode = rosNode;
    std::string pubPrefix = "rm_driver" + std::to_string(mArmNumber);
    std::string subPrefix = "/rm_driver" + std::to_string(mArmNumber);

    std::string topicMoveJ = pubPrefix + "/MoveJ_Cmd";
    std::string topicToolDOutput = pubPrefix + "/Tool_Digital_Output";
    std::string topicToolAOutput = pubPrefix + "/Tool_Analog_Output";
    mPubMoveJ = mRosNode.advertise<rm_msgs::MoveJ>(topicMoveJ, 1000);
    // mPubMoveJ = mRosNode.advertise<rm_msgs::MoveJ>("rm_driver1/MoveJ_Cmd", 1000);
    mPubToolDOutput = mRosNode.advertise<rm_msgs::Tool_Digital_Output>(topicToolDOutput, 1000);
    mPubToolAOutput = mRosNode.advertise<rm_msgs::Tool_Analog_Output>(topicToolAOutput, 1000);

    std::string topicPlanState = subPrefix + "/Plan_State";
    mSubPlanState = mRosNode.subscribe(topicPlanState, 1000, &ArmController::PlanStateCallback, this);
    // mSubPlanState = mRosNode.subscribe("/rm_driver1/Plan_State", 1000, &ArmController::PlanStateCallback, this);
}

/**
 * @brief 更新机械臂配置
 * 
 * @param armConfigs
 */
void ArmController::updateMoveJ(std::vector<ArmConfig> armConfigs){
    if (armConfigs.empty())
        return;
    unique_lock<mutex> lock(mutexArmConfigs);
    mArmConfigs.clear();
    for (auto config:armConfigs){
        ArmConfig config_copy = config;
        mArmConfigs.push_back(config_copy);
    }
    printNum();
    printf("!!! updateMoveJ 成功\n");
    mtpubMoveJ = new thread(&ArmController::pubMoveJ, this);
    mtpubMoveJ->detach();
}

void ArmController::updateHand(bool handState){
    unique_lock<mutex> lock(mutexHand);
    mHandState = handState;
    mtpubHand = new thread(&ArmController::pubHand, this);
}

void ArmController::pubMoveJ(){
    unique_lock<mutex> lock(mutexArmConfigs);
    if(!mArmConfigs.empty()){
        rm_msgs::MoveJ msg;
        auto config = mArmConfigs[0];

        if (config.mPauseTime)
            std::this_thread::sleep_for(std::chrono::milliseconds(config.mPauseTime));
        cout << "[" ;
        for(int i=0;i<6;i++){
            msg.joint[i] = config.jointPos[i];
            // msg.joint[i] = 0.0;
            cout << msg.joint[i] << ", ";
        }
        cout << "]\n";
        msg.speed = config.speed;

        printNum();
        printf("!!! mPubMoveJ.publish 成功\n");
        mPubMoveJ.publish(msg);
    }
}

void ArmController::pubHand()
{
    unique_lock<mutex> lock(mutexHand);

    ros::Rate loop_rate(3);
    rm_msgs::Tool_Digital_Output DO_msg;
    rm_msgs::Tool_Analog_Output AO_msg;

    //　张开手
    if (mHandState) {
        DO_msg.num = 2;
        DO_msg.state = true;
        AO_msg.voltage = 0.0;

        for(int i = 0 ; i < 2 ; i++){
            mPubToolAOutput.publish(AO_msg);
            loop_rate.sleep();
            mPubToolDOutput.publish(DO_msg);
        }
        FABO_ROBOT::printInColor("Successfully open hand of ", FABO_ROBOT::GREEN);
        printNum();
    }
    else{
        DO_msg.num = 2;
        DO_msg.state = false;
        AO_msg.voltage = 10.0;
        for(int i = 0 ; i < 2 ; i++){
            mPubToolDOutput.publish(DO_msg);
            loop_rate.sleep();
            mPubToolAOutput.publish(AO_msg);
        }
        FABO_ROBOT::printInColor("Successfully hold hand of ", FABO_ROBOT::GREEN);
        printNum();
    }
}

void ArmController::PlanStateCallback(const rm_msgs::Plan_State &msg){
    unique_lock<mutex> lock(mutexArmConfigs);
    printNum();
    FABO_ROBOT::printInColor("!!! PlanStateCallback 成功\n\n", FABO_ROBOT::BLUE);
    if (msg.state){
        if(!mArmConfigs.empty()){
            mArmConfigs.erase(mArmConfigs.begin());
        }
    }
    mtpubMoveJ = new thread(&ArmController::pubMoveJ, this);
    mtpubMoveJ->detach();
}

void ArmController::printNum(){
    cout << "ArmNum " << mArmNumber << " ";
}