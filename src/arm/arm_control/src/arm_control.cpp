//
// Created by lj on 2022/3/18.
//

#include "string"
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "arm_control/Hand_Control.h"
#include <rm_msgs/Tool_Analog_Output.h>
#include <rm_msgs/Tool_Digital_Output.h>
#include <rm_msgs/MoveJ.h>
#include <arm_control/Arms.h>

#include "std_msgs/String.h"

#include "ArmsActionManager.h"


class ArmControl
{
public:
    ArmControl(ros::NodeHandle& n);
    void readinArmActions(const string &config_file){
        action_manager = new ArmsActionManager(config_file);
    };
    void callAction(const arm_control::Arms &msg);
private:
    ros::NodeHandle n_;
    ros::Subscriber subscriber_arm_;
    ros::ServiceServer service_hold_hand_;
    ros::ServiceServer service_open_hand_;
    ros::Publisher d_output_pub_;
    ros::Publisher a_output_pub_;
    ros::Publisher arm1_action_pub_;
    ros::Publisher arm2_action_pub_;
    ArmsActionManager *action_manager;
    void arm_action_callback(const arm_control::Arms &msg);
    bool hand_callback(arm_control::Hand_Control::Request &req,
                            arm_control::Hand_Control::Response &res);
    // bool open_hand_callback(arm_control::Hand_Control::Request &req,
    //                         arm_control::Hand_Control::Response &res);
    void singleArmActionPub(ros::Publisher, std::vector<ArmConfig>);
};

ArmControl::ArmControl(ros::NodeHandle& n)
{
    n_ = n;
    subscriber_arm_ = n_.subscribe("/arm_action", 1000, &ArmControl::arm_action_callback, this);
    service_hold_hand_ = n_.advertiseService("hand_control", &ArmControl::hand_callback,this);
    arm1_action_pub_ = n_.advertise<rm_msgs::MoveJ>("rm_driver1/MoveJ_Cmd", 1000);
    arm2_action_pub_ = n_.advertise<rm_msgs::MoveJ>("rm_driver2/MoveJ_Cmd", 1000);
    ROS_INFO("Ready to hold or open hands.");
}

void ArmControl::callAction(const arm_control::Arms &msg){
    arm_action_callback(msg);
}

void ArmControl::arm_action_callback(const arm_control::Arms &msg)
{
    const auto action_library = action_manager->get_arms_actions();
    auto action_index = action_library.find(msg.action);

    if(action_index != action_library.end()){
        cout << "Receive arm_action order: " << msg.action << endl;

        auto l_arm_action = action_index->second.left_arm_action;
        auto r_arm_action = action_index->second.right_arm_action;

        std::thread left_arm_thread(&ArmControl::singleArmActionPub,this,arm1_action_pub_,l_arm_action);
        std::thread right_arm_thread(&ArmControl::singleArmActionPub,this,arm2_action_pub_,r_arm_action);

        left_arm_thread.join();
        right_arm_thread.join();
        cout << "Complete arm_action :" << msg.action << endl;
        cout << "----------------------------------------\n";
        return;
    }
    else{
        cout << "Error: unavailable arm_action " << msg.action << endl;
        return;
    }
}

void ArmControl::singleArmActionPub(ros::Publisher publisher,std::vector<ArmConfig> single_arm_actions)
{
    rm_msgs::MoveJ msg;
    for(auto &arm_config:single_arm_actions){
        cout << "[" ;
        for(int i=0;i<6;i++){
            msg.joint[i] = arm_config.jointPos[i];
            cout << msg.joint[i] << ", ";
        }
        cout << "]\n";
        msg.speed = arm_config.speed;
        publisher.publish(msg);
        cout << "wait" << endl;
        arm_config.pause_time->sleep();
    }
}

bool ArmControl::hand_callback(arm_control::Hand_Control::Request &req,
                               arm_control::Hand_Control::Response &res)
{
    
    if(req.Arm_Number==1 || req.Arm_Number==2){
        std::string tool_a_output_str = "rm_driver" + std::to_string(req.Arm_Number) + "/Tool_Analog_Output";
        std::string tool_d_output_str = "rm_driver" + std::to_string(req.Arm_Number) + "/Tool_Digital_Output";

        a_output_pub_ = n_.advertise<rm_msgs::Tool_Analog_Output>(tool_a_output_str, 1000);
        d_output_pub_ = n_.advertise<rm_msgs::Tool_Digital_Output>(tool_d_output_str, 1000);

        ros::Rate loop_rate(3);
        rm_msgs::Tool_Digital_Output msg_d;
        rm_msgs::Tool_Analog_Output msg_a;

        msg_d.num = 2;
        msg_d.state = req.is_open;
        msg_a.voltage = 0.0;

        if(req.is_open){
            for(int i=0;i<2;i++){
                a_output_pub_.publish(msg_a);
                loop_rate.sleep();
                d_output_pub_.publish(msg_d);
            }
            ROS_INFO("Successfully open the hand of arm.");
        }
        else{
            for(int i=0;i<2;i++){
                d_output_pub_.publish(msg_d);
                loop_rate.sleep();
                a_output_pub_.publish(msg_a);
            }
            ROS_INFO("Successfully hold the hand of arm.");
        }
        res.has_finished = true;
        return true;
    }
    else{
        ROS_INFO("Wrong Arm_Number in ros service /hand_Control.");
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_control");
    ros::NodeHandle n;
    string config_file = "/home/lj/Documents/Indoor-mobile-robot-with-arms/src/arm/arm_control/json/arms_actions.json";
    // string config_file = "/home/lj/Documents/Indoor-mobile-robot-with-arms/src/arm/arm_control/json/try.json";
    ArmsActionManager arms_action_manager(config_file);
    
    // 创建机械臂控制实例
    ArmControl arm_control(n);
    // 加入动作
    arm_control.readinArmActions(config_file);
    
    sleep(2);

    arm_control::Arms msg;
    string action_str = "sleep";
    msg.action = action_str;
    arm_control.callAction(msg);

    ros::spin();

    return 0;
}