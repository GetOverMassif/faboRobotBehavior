//
// Created by lj on 2022/3/18.
//

#include "string"
#include <iostream>
#include "ros/ros.h"
#include "arm_control/Hand_Control.h"
#include <rm_msgs/Tool_Analog_Output.h>
#include <rm_msgs/Tool_Digital_Output.h>

#include "std_msgs/String.h"

// 1

class ArmControl
{
public:
    ArmControl(ros::NodeHandle& n);
private:
    ros::NodeHandle n_;
    ros::ServiceServer service_hold_hand_;
    ros::ServiceServer service_open_hand_;
    ros::Publisher d_output_pub_;
    ros::Publisher a_output_pub_;
    bool hold_hand_callback(arm_control::Hand_Control::Request &req,
                            arm_control::Hand_Control::Response &res);
    bool open_hand_callback(arm_control::Hand_Control::Request &req,
                            arm_control::Hand_Control::Response &res);
};

ArmControl::ArmControl(ros::NodeHandle& n)
{
    n_ = n;

    service_hold_hand_ = n_.advertiseService("hand_control/hold", &ArmControl::hold_hand_callback,this);
    service_open_hand_ = n_.advertiseService("hand_control/open", &ArmControl::open_hand_callback,this);
    ROS_INFO("Ready to hold or open hands.");
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