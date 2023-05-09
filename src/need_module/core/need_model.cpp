//
// Created by zhjd on 2021/5/11.
//
//ros头文件
#include "common_include.h"

//内部头文件
// #include "need_satisfied.h"
#include "prior_need.h"
#include "perception_filter.h"

using namespace  std;
// time_t inner_need::time_for_wandor  =  0;
perception_filter *Filter = new perception_filter(20);   //TODO: 确定合适的per过滤时长阈值。

// ros node
ros::Subscriber sub_perception;
ros::Subscriber sub_robot_emotion;
ros::Subscriber sub_robot_status;
ros::Publisher pub;  //need
ros::Publisher query_attitude;
ros::Subscriber sub_idleState;
//全局变量: 
int period_cur = 0;   //每个period的周期时长,由 sleep函数 决定。
social_msg::attitude_msg  attitude_query_result; 

//运算符
void operator >> (const social_msg::need_msg& msg, need_wu & need) {
    need.need_name = msg.need_name;
    need.IDtype = msg.IDtype;
    need.weight = msg.weight;
}

// 需求模型
prior_need PriorNeed;
void run_PriorNeed(ros::NodeHandle*  n_ptr);


// 回调函数
void PerceptionUpdate(const social_msg::perception_msg& msg){
    
    // 旧版
    // perception per;
    // per.intention_ = msg.intention;
    // per.p_ = msg.p;
    // per.intention_2_ = msg.intention_2;
    // if(  msg.intention_2 == ""    || msg.p_2 == 0  || msg.p_2 == NULL ){
    //     per.p_2_ = 0;
    // }
    // else{
    //     per.p_2_ = msg.p_2;
    // }
    // per.person_name_ = msg.person_name;
    // per.IDtype_ = msg.IDtype;
    
    // per.speech_ = msg.speech;
    // per.person_emotion_ = msg.person_emotion;
    
    // 新版
    social_msg::perception_msg per;
    per = msg;
    if(  msg.intention_2 == "" || msg.p_2 == 0 || msg.p_2 == NULL ){
        per.p_2 = 0;
    }
    else{
        per.p_2 = msg.p_2;
    }


    if( Filter->Whether_OK(per) )    //如果,“感知过滤器”认为当前感知是有效的,则update
        PriorNeed.PerceptionUpdate(per);

    
    // // 【暂时】需求满足节点,生成need_satisfied
    // need_satisfied.perception_to_needSatisfied( per );
}

void RobotEmotionUpdate(const social_msg::robot_emotion& msg){
    
    double emotion[8];
    emotion[0] = msg.emotion1;
    emotion[1] = msg.emotion2;
    emotion[2] = msg.emotion3;
    emotion[3] = msg.emotion4;
    emotion[4] = msg.emotion5;
    emotion[5] = msg.emotion6;
    emotion[6] = msg.emotion7;
    emotion[7] = msg.emotion8;
    PriorNeed.RobotEmotionUpdate(emotion);  
    // adaptiveModel.RobotEmotionUpdate(emotion);
}

void RobotStatusUpdate(const social_msg::robot_status& msg){
    
    double status[8] ;
    status[0] = msg.body1;
    status[1] = msg.body2;
    status[2] = msg.body3;
    status[3] = msg.body4;
    status[4] = msg.body5;
    status[5] = msg.body6;
    status[6] = msg.body7;
    status[7] = msg.idleState;  
    PriorNeed.RobotStatusUpdate(status);
}

// void BehaviorUpdate(const social_msg::bhvPara::ConstPtr& behavior_ ,  ros::NodeHandle*  n){
void BehaviorFinishedUpdate(const social_msg::idleState::ConstPtr& msg,  ros::NodeHandle*  n_ptr){
    
    if(msg->hehavior_name == "MeasureTempareture"){
        social_msg::need_msg need_output;
        need_output.person_name =  msg->person_name;
        need_output.scene = "school";
        need_output.IDtype = msg->IDtype;
        need_output.need_name = "TellTemparetureResult";  
        need_output.target_angle = msg->target_angle;
        need_output.target_distance = msg->target_distance;
        need_output.rob_emotion = "Joy";//TODO: need_lists[i].rob_emotion;
        need_output.rob_emotion_intensity = 2;
        need_output.person_emotion = msg->person_emotion;//need_lists[i].person_emotion
        need_output.weight = 1.0;
        need_output.speech = "";
        need_output.qt_order = period_cur;
        need_output.satisfy_value = 1.0;
        
        //接受社交态度及相关参数
        social_msg::attitude_query query;
        query.IDtype = need_output.IDtype;
        query.motivation = need_output.need_name;
        query.person_name = need_output.person_name;
        query_attitude.publish(query);
        ros::Duration timeout(0.5);
        social_msg::attitude_msg::ConstPtr result = ros::topic::waitForMessage<social_msg::attitude_msg>("attitude", *n_ptr, timeout);
        if (result != NULL)
            {
                if( need_output.person_name ==  result->person_name &&
                    need_output.IDtype      ==  result->IDtype &&
                    need_output.need_name   ==  result->motivation
                    ){
                    ROS_INFO("Received right social attitude");
                    need_output.attitude    = result->attitude;
                    need_output.move_speed  = result->move_speed;
                    need_output.distance   = result->distance;
                    need_output.voice_speed = result->voice_speed;
                }
                else{
                    ROS_INFO("Received wrong social attitude");
                    need_output.attitude    = "";
                    need_output.move_speed  = 0;
                    need_output.distance    = 0;
                    need_output.voice_speed = 0;
                }
            }
            else
            {
                ROS_WARN("Timeout social attitude within 0.5 seconds");
            }   
        pub.publish(need_output);
        std::cout <<  "    Output Need " << 0+1 << ": " << need_output.need_name << " ,Weight: " <<need_output.weight;
        if (  need_output.person_name != "")
            std::cout <<" ,for " <<need_output.person_name<<" as " <<need_output.IDtype;
        std::cout<<std::endl;
    }
}

int main(int argc, char** argv){
    // ROS
    ros::init(argc, argv, "need_module");
    ros::NodeHandle n;
    cout<< "Start to Subscribe（接收ROS信息） !!\n";
    
    //状态更新 
    sub_perception = n.subscribe("perceptions", 1000, PerceptionUpdate);
    sub_robot_emotion = n.subscribe("robot_emotion", 1000, RobotEmotionUpdate);
    sub_robot_status = n.subscribe("robot_status", 1000, RobotStatusUpdate);
    sub_idleState = n.subscribe<social_msg::idleState>("idleState", 1000,   boost::bind(&BehaviorFinishedUpdate, _1, &n));
    
    ros::spinOnce();
    
    // 需求发布
    pub = n.advertise<social_msg::need_msg>("need_lists", 10);  

    // 社交态度查询
    query_attitude = n.advertise<social_msg::attitude_query>("attitude_query", 1);  

    // 控制需求先验模型的运行周期
    // ros::Rate loop_rate(0.1);  //5s一次
    ros::Rate loop_rate(0.6);  //5s一次

    // 为需求模型的运行  创建单独的线程 。  
    // std::thread PriorNeedThread(run_PriorNeed);
    cout<< "Wait to run PriorNeed !!\n";
    while(ros::ok){
        if( ros::isShuttingDown() )
            break;
        run_PriorNeed(&n);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    // ros::spin();    //库是节点读取数据道消息响应循环,当消息到达的时候,回调函数就会被调用。当按下Ctrl+C时,节点会退出消息循环,于是循环结束。
    return 0;
}

int qt_order_debug = 0;
void run_PriorNeed(ros::NodeHandle*  n_ptr){
    // while(1)
    {
        if(PriorNeed.updateInit())
        {
            printf( GREEN "Run %dth PriorNeed（运行先验模型） !!\n"NONE, period_cur);            
            vector<need> need_lists = PriorNeed.need_compute_all();
            if( need_lists.size() != 0 )
                for(int j =0 ; j< need_lists.size(); j++){
                    
                    //接受社交态度及相关参数
                    social_msg::attitude_query query;
                    query.IDtype = need_lists[j].IDtype;
                    query.motivation = need_lists[j].need_name;
                    query.person_name = need_lists[j].person_name;
                    query_attitude.publish(query);
                    ros::Duration timeout(0.5);
                    social_msg::attitude_msg::ConstPtr result = ros::topic::waitForMessage<social_msg::attitude_msg>("attitude", *n_ptr, timeout);

                    // 将need发布给行为模块
                    social_msg::need_msg need_output;
                    need_output.person_name =  need_lists[j].person_name;
                    need_output.IDtype = need_lists[j].IDtype;
                    need_output.need_name = need_lists[j].need_name;  
                    need_output.target_angle = need_lists[j].target_angle;
                    need_output.target_distance = need_lists[j].target_distance;
                    need_output.rob_emotion = need_lists[j].robot_emotion_str;//TODO: need_lists[i].rob_emotion;
                    need_output.rob_emotion_intensity = need_lists[j].robot_emotion_intensity;
                    need_output.person_emotion = need_lists[j].person_emotion;//need_lists[i].person_emotion
                    need_output.weight = need_lists[j].weight;
                    need_output.speech = need_lists[j].speech;
                    need_output.qt_order = period_cur;
                    need_output.satisfy_value = need_lists[j].satisfy_value;
                    if (result != NULL)
                    {
                        if( need_output.person_name ==  result->person_name &&
                            need_output.IDtype      ==  result->IDtype &&
                            need_output.need_name   ==  result->motivation
                         ){
                            ROS_INFO("Received right social attitude");
                            need_output.attitude    = result->attitude;
                            need_output.move_speed  = result->move_speed;
                            need_output.distance   = result->distance;
                            need_output.voice_speed = result->voice_speed;
                        }
                        else{
                            ROS_INFO("Received wrong social attitude");
                            need_output.attitude    = "";
                            need_output.move_speed  = 0;
                            need_output.distance    = 0;
                            need_output.voice_speed = 0;
                        }
                    }
                    else
                    {
                        ROS_WARN("Timeout: Failed to receive social attitude within 0.5 seconds");
                    }   
                    pub.publish(need_output);
                    // printf( GREEN "    QT_order: %d:\n"NONE, need_output.qt_order); 
                    // sleep(0.1); // TODO: 重要。
                }
            else{  //此部分用途: 用于刷新qt中need list。如果本周期,没有新的need发送过,那么qt中的need list还是会显示之前的need,就可能会对中期测试中本没有need生成的情况  造成误解。
                    // social_msg::need_msg need_output;
                    // need_output.IDtype = "";
                    // need_output.rob_emotion = "";//TODO: need_lists[i].rob_emotion;
                    // need_output.person_emotion = "";//need_lists[i].person_emotion
                    // need_output.need_name = "";  
                    // need_output.weight = 0;
                    // need_output.speech = "";
                    // need_output.person_name =  "";
                    // need_output.qt_order = period_cur;
                    // need_output.satisfy_value = 0;
                    // pub.publish(need_output);
            }
            period_cur++;     
        }
    }
    // cout<< "End PriorNeed !!\n";
}


