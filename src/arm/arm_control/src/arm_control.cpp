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

// 1
// 2
// 3

class ArmControl
{
public:
    ArmControl(ros::NodeHandle& n);
private:
    ros::NodeHandle n_;
    ros::Subscriber subscriber_arm_;
    ros::ServiceServer service_hold_hand_;
    ros::ServiceServer service_open_hand_;
    ros::Publisher d_output_pub_;
    ros::Publisher a_output_pub_;
    ros::Publisher arm_action_pub_;
    void arm_action_callback(const arm_control::Arms &msg);
    bool hold_hand_callback(arm_control::Hand_Control::Request &req,
                            arm_control::Hand_Control::Response &res);
    bool open_hand_callback(arm_control::Hand_Control::Request &req,
                            arm_control::Hand_Control::Response &res);
};

ArmControl::ArmControl(ros::NodeHandle& n)
{
    n_ = n;
    subscriber_arm_ = n_.subscribe("/arm_action", 1000, &ArmControl::arm_action_callback, this);
    service_hold_hand_ = n_.advertiseService("hand_control/hold", &ArmControl::hold_hand_callback,this);
    service_open_hand_ = n_.advertiseService("hand_control/open", &ArmControl::open_hand_callback,this);
    arm_action_pub_ = n_.advertise<rm_msgs::MoveJ>("rm_driver2/MoveJ_Cmd", 1000);
    ROS_INFO("Ready to hold or open hands.");
}

void ArmControl::arm_action_callback(const arm_control::Arms &msg)
{
    if(msg.action=="wave"){
        ROS_INFO("Receive Arm.msg.action = 'wave'.");
        rm_msgs::MoveJ msg;
        ros::Rate loop_rate(0.9);
        std::vector<std::vector<float>> action_value;
        action_value.push_back(std::vector<float>{1.45, -0.40, 0.03, -0.12, -0.11, 0.30, 0.5});
        action_value.push_back(std::vector<float>{1.45, -0.17, 0.45, -0.11, 0.38, 0.55, 0.5});
        action_value.push_back(std::vector<float>{1.45, -0.40, 0.03, -0.12, -0.11, 0.30, 0.5});
        action_value.push_back(std::vector<float>{1.45, -0.17, 0.45, -0.11, 0.38, 0.55, 0.5});
        action_value.push_back(std::vector<float>{1.45, -0.40, 0.03, -0.12, -0.11, 0.30, 0.5});
        action_value.push_back(std::vector<float>{1.45, -0.17, 0.45, -0.11, 0.38, 0.55, 0.5});
        for(auto &single_action:action_value)
        {
            msg.joint[0] = single_action.at(0);
            msg.joint[1] = single_action.at(1);
            msg.joint[2] = single_action.at(2);
            msg.joint[3] = single_action.at(3);
            msg.joint[4] = single_action.at(4);
            msg.joint[5] = single_action.at(5);
            msg.speed = single_action.at(6);
            arm_action_pub_.publish(msg);
            loop_rate.sleep();
        }
    }
    else{
        return;
    }
}

bool ArmControl::hold_hand_callback(arm_control::Hand_Control::Request &req,
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
        msg_d.state = false;
        msg_a.voltage = 10.0;

        for(int i=0;i<2;i++)
        {
            d_output_pub_.publish(msg_d);
            loop_rate.sleep();
            a_output_pub_.publish(msg_a);
        }

        ROS_INFO("Successfully hold the hand of arm.");

        res.has_finished = true;
    }
    else{
        ROS_INFO("Wrong Arm_Number in ros service /hand_Control/hold.");
        return false;
    }
    return true;
}

bool ArmControl::open_hand_callback(arm_control::Hand_Control::Request &req,
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
        msg_d.state = true;
        msg_a.voltage = 0.0;

        for(int i=0;i<2;i++)
        {
            a_output_pub_.publish(msg_a);
            loop_rate.sleep();
            d_output_pub_.publish(msg_d);
        }

        ROS_INFO("Successfully open the hand of arm.");

        res.has_finished = true;
    }
    else{
        ROS_INFO("Wrong Arm_Number in ros service /hand_Control/open.");
        return false;
    }
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_control");
    ros::NodeHandle n;

    ArmControl arm_control(n);

    ros::spin();

    return 0;
}