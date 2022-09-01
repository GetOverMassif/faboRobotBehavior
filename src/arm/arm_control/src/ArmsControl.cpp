#include "ArmsControl.h"

ArmsControl::ArmsControl(ros::NodeHandle& n)
{
    n_ = n;
    subscriber_arm_ = n_.subscribe("/arm_action", 1000, &ArmsControl::arm_action_callback, this);
    service_hold_hand_ = n_.advertiseService("hand_control", &ArmsControl::hand_callback,this);
    ROS_INFO("Ready to hold or open hands.");

    mLeftArm = new ArmController(n_, 1);
    mRightArm = new ArmController(n_, 2);
}

void ArmsControl::readinArmActions(const string &config_file){
    action_manager = new ArmsActionManager(config_file);
};

void ArmsControl::callAction(const arm_control::Arms &msg){
    arm_action_callback(msg);
}

void ArmsControl::arm_action_callback(const arm_control::Arms &msg)
{
    const auto action_library = action_manager->get_arms_actions();
    auto action_index = action_library.find(msg.action);

    if(action_index != action_library.end()){
        cout << "Receive arm_action order: " << msg.action << endl;

        auto l_arm_action = action_index->second.left_arm_action;
        auto r_arm_action = action_index->second.right_arm_action;

        // mLeftArm->updateMoveJ(l_arm_action);
        // printf("\n!!!mRightArm开始发送指令\n");
        // mRightArm->updateMoveJ(r_arm_action);

        mRightArm->updateMoveJ(r_arm_action);
        printf("\n!!!mRightArm开始发送指令\n");
        mLeftArm->updateMoveJ(l_arm_action);

        cout << "Complete arm_action :" << msg.action << endl;
        cout << "----------------------------------------\n";
        return;
    }
    else{
        cout << "Error: unavailable arm_action " << msg.action << endl;
        return;
    }
}

bool ArmsControl::hand_callback(arm_control::Hand_Control::Request &req,
                               arm_control::Hand_Control::Response &res)
{
    
    if(req.Arm_Number==1 || req.Arm_Number==2){
        if (req.Arm_Number == 1) {
            mLeftArm->updateHand(req.is_open);
        }
        else{
            mRightArm->updateHand(req.is_open);
        }
        res.has_finished = true;
        return true;
    }
    else{
        ROS_INFO("Wrong Arm_Number in ros service /hand_Control.");
        return false;
    }
}