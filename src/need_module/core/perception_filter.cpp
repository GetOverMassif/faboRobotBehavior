/*
 * @Descripttion: 
 * @version: 
 * @Author: sueRimn
 * @Date: 2021-09-21 21:16:50
 * @LastEditors: Zhang Jiadong
 * @LastEditTime: 2023-04-25 15:42:15
 */
#include "perception_filter.h"

std::vector<social_msg::perception_msg>  perception_filter::per_list;


bool perception_filter::Whether_OK( social_msg::perception_msg per) {
        
        // 检测是否 重复;, 
        // 内容是完备
        if( per.intention == "" ||  per.p  == NULL || per.person_name =="" || per.IDtype ==""){
            return false;  /* intention为空的话,不传入prior need即可。 */
        }
        //  时间  如果时间差小于阈值,则false； 如果大于阈值,说明很长时间没收到这类percepiton了,则true；
        // for(auto iter = per_list.end(); iter != per_list.begin(); iter-- ){
        //     if(
        //         iter->person_name_ == per.person_name_  &&  iter->IDtype_ == per.IDtype_ &&  
        //         iter->person_name_ == per.person_name_ &&  iter->intention_ == per.intention_  &&  
        //         iter->speech_ == per.speech_   )
        //     {
        //         double diff = abs(iter->time_ - per.time_);

        //         if( diff < time_thresh)  return false; 
        //     }
        // }
        per_list.push_back( per );
        return true;
    }

