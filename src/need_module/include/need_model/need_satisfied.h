/*
 * @Descripttion: 
 * @version: 
 * @Author: Zhang Jiadong
 * @Date: 2022-05-08 20:24:12
 * @LastEditors: Zhang Jiadong
 * @LastEditTime: 2022-05-10 20:36:59
 */
#include "common_include.h"
#include <ros/ros.h>

class NeedSatisfied{
    private:   
        ros::Publisher pub;
        std::vector<need> need_already_pub_list;
        need need_cur;
        need need_old;
    public:
        NeedSatisfied(){
            need_old.need_name = "";
            need_old.person_name = "";
            need_cur.need_name = "";
            need_cur.person_name = "";
        }
        
        // need_satisfied(ros::NodeHandle& n){
        //     pub = n.advertise<social_msg::need_satisfy_msg>("need_satisfied", 10); 
        // }
        void set_ros_node(ros::NodeHandle& n){
            pub = n.advertise<social_msg::need_satisfy_msg>("need_satisfied", 10);  
        }
    
        // 建立已有的 need vector
        bool insert_need_list( need need_new){
            need_already_pub_list.push_back(need_new);
        }
        
        // 当回调函数中接收到 行为process后,对need vector进行管理。
        // 当前: 如果 收到了某几个特定的感知信息,则生成 need_satisfy_msg。 比如扰乱秩序。
        void perception_to_needSatisfied( const perception &p){
            social_msg::need_satisfy_msg msg;
            if( p.intention_ == "Uncooperate"){
                msg.need_name = "MeasureTempareture";
                msg.satisfy_value = -2;
                pub.publish(msg);
                printf(  BLUE "     NeedSatisfied for MeasureTempareture \n"NONE);
            } 
        }

        // 真正的: 如果有需求被打断,则认为 need么有被满足。方法: 
        // （1）根据行为process,确定当前执行的 need_cur。
        // （2）如果下一个 行为process对应的need 与 need_cur不同,则说明被打断。生成 负向的need_satisfy_msg
        // （3）如果行为process到达100%, 则说明需求被满足,生成 正向的need_satisfy_msg。还要把这个从need vector中剔除。
        void behaviorProcess_to_needSatisfied(const social_msg::bhvReply & process){
            
        }
};