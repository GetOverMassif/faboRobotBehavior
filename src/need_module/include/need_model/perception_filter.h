/*
 * @Descripttion: 
 * @version: 
 * @Author: sueRimn
 * @Date: 2021-09-21 21:16:50
 * @LastEditors: Zhang Jiadong
 * @LastEditTime: 2023-04-25 15:40:31
 */
#ifndef TEST_FILTER_H
#define TEST_FILTER_H

#include "common_include.h"

class perception_filter{
public:
    double time_;
    static std::vector<social_msg::perception_msg>  per_list;   /* 如何让这个list  始终不变。 static？？ */
    social_msg::perception_msg per;
    double time_thresh;
public:
    perception_filter(){};
    perception_filter( double time ): time_thresh(time){};
    bool Whether_OK( social_msg::perception_msg per) ;
    
};

#endif //TEST_PERCEPTION_H


// 意图过滤
// bool Whether_Map(perception & perception ){
//     for( auto  iter = intention_list.begin(); iter != intention_list.end(); iter ++ )
//         if ( iter->person_ == perception.person_  &&     iter->intention_ == perception.intention_)
//         //    如果时间间隔都大于阈值,则执行映射。只要有小于的,就不执行。
//             if(abs(iter->time_ - perception.time_) < threshold_time)
//                 return false;
//             else
//                 return true;
// }

// “巡逻”  是一定要发送的需求。因此在need lists中,先添加这个。 todo: 说明需求是有触发条件的,绝大多数需求的触发条件是“意图”,但是巡逻的触发条件是“空闲”。因此不好表达。

// // 不需要了: 意图筛选。对短时间内重复的意图,进行剔除  lautering; colating;  维护一个list。（时间、）
// void intentionlist_filter(perception & perception_now){
// // 错误: 如果时间大于阈值,说明新来的重复意图,应该取代旧的重复意图。这样,intention list中就永远是不同的意图,并且各自对应的时间都是最新的时间。从而,利用intentionlist看看新来的意图,要不要进行映射。
// // 
//     for( auto  iter = intention_list.begin(); iter != intention_list.end(); iter ++ )
//         if ( iter->person_ == perception_now.person_  &&     iter->intention_ == perception_now.intention_)
//             if(abs(iter->time_ - perception_now.time_) > threshold_time)
//             {
//                 intention_list.erase( iter );
//                 intention_list.push_back( perception_now );
//             }
// }
