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
    ros::Publisher arm1_action_pub_;
    ros::Publisher arm2_action_pub_;
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
    arm1_action_pub_ = n_.advertise<rm_msgs::MoveJ>("rm_driver1/MoveJ_Cmd", 1000);
    arm2_action_pub_ = n_.advertise<rm_msgs::MoveJ>("rm_driver2/MoveJ_Cmd", 1000);
    ROS_INFO("Ready to hold or open hands.");
}

void ArmControl::arm_action_callback(const arm_control::Arms &msg)
{
    if(msg.action=="wave"){
        ROS_INFO("Receive Arm.msg.action = 'wave'.");
        rm_msgs::MoveJ msg1,msg2;
        ros::Rate loop_rate(1.2);
        std::vector<std::vector<float>> arm1_action_value;
        std::vector<std::vector<float>> arm2_action_value;
        arm2_action_value.push_back(std::vector<float>{-0.57, 1.09, 1.65, -2.04, -1.24, 0.63, 0.6});
        arm2_action_value.push_back(std::vector<float>{1.65, 0.70, 0.61, -0.07, 0.04, 0.46, 0.8});
        arm2_action_value.push_back(std::vector<float>{1.69, 0.80, 1.05, 0.09, 0.42, 0.62, 0.8});
        arm2_action_value.push_back(std::vector<float>{1.65, 0.70, 0.61, -0.07, 0.04, 0.46, 0.8});
        arm2_action_value.push_back(std::vector<float>{1.69, 0.80, 1.05, 0.09, 0.42, 0.62, 0.8});
        arm2_action_value.push_back(std::vector<float>{1.65, 0.70, 0.61, -0.07, 0.04, 0.46, 0.8});
        arm2_action_value.push_back(std::vector<float>{1.69, 0.80, 1.05, 0.09, 0.42, 0.62, 0.8});
        arm2_action_value.push_back(std::vector<float>{-0.57, 1.09, 1.65, -2.04, -1.24, 0.63, 0.6});
        for(auto &single_action:arm2_action_value)
        {
            for(int i=0;i<6;i++){
                msg2.joint[i] = single_action.at(i);
            }
            msg2.speed = single_action.at(6);
            arm2_action_pub_.publish(msg);
            loop_rate.sleep();
        }
    }
    else if(msg.action=="Curl"){
        ROS_INFO("Receive Arm.msg.action = 'Curl'.");
        rm_msgs::MoveJ msg1,msg2;
        ros::Rate loop_rate(0.8);
        std::vector<std::vector<float>> arm1_action_value;
        std::vector<std::vector<float>> arm2_action_value;
        arm1_action_value.push_back(std::vector<float>{0.57, -1.09, -1.65, 2.04, 1.24, -0.63, 0.6});
        arm2_action_value.push_back(std::vector<float>{-0.57, 1.09, 1.65, -2.04, -1.24, 0.63, 0.6});
        for(int i=0;i<6;i++){
            msg1.joint[i] = arm1_action_value.at(0).at(i);
            msg1.joint[i] = arm1_action_value.at(0).at(i);
        }
        msg1.speed = arm1_action_value.at(0).at(6);
        msg2.speed = arm2_action_value.at(0).at(6);
        arm1_action_pub_.publish(msg1);
        arm2_action_pub_.publish(msg2);
        loop_rate.sleep();
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