//
// Created by zhjd on 2021/5/11.
//
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include "task_need.h"
#include "inner_need.h"
#include "emergency.h"

#ifndef TEST_CHECKRULE_H
#define TEST_CHECKRULE_H

//std::vector< std::string > need_list    = { "greet_teacher", "greet_student", "greet_parent"};
//std::vector< int > weight_list          = {5,4,3,2,1};



class prior_need {
private:

    /* 任务 类 */
    task_need *task_model =  new task_need();;
    /* 生理 类 */
    inner_need *inner_model = new inner_need();
    /* 应急 类 */
    Emergency *emergency_model = new Emergency(); 
    /* 结构体 ,用于发送给行为管理*/
    
    social_msg::perception_msg per_;
    social_msg::perception_msg per_second;
    std::vector<social_msg::perception_msg> per_list ;
    bool first_per_undeal = false;
    double emotion_[8];
    double body_[8] =  {1, 1, 1, 1, 1, 1, 1, 0};
    int need_num;
    bool updateInit_perception = false;
    bool updateInit_emotion = false;
    bool updateInit_status = false;

private:
    double varianceTop2( social_msg::perception_msg & p ){
        
        double mean = p.p/2.0 + p.p_2/2.0;
        // double var =   ( (p.p_/2.0-mean)*(p.p_/2.0-mean)  + (p.p_2_/2.0-mean)*(p.p_2_/2.0-mean) ) /2.0;
        double var = pow(p.p-mean,2) + pow(p.p_2-mean,2);
        return var/2.0;
    }

public:   
    prior_need(  ) {
        perceptionClear();
     }
    
    void PerceptionUpdate(social_msg::perception_msg per ){ 

        per_list.push_back( per );
        printf(  YELLOW "     Update Peception: "NONE);
        cout<< per.intention <<" and "<<per.intention_2<< endl;
        updateInit_perception = true;
    }

    void RobotEmotionUpdate(double emotion[8] ){ 
        printf(  YELLOW "     Update Robot Emotion: "NONE);  
        for(int i = 0 ; i < 8 ; i ++ ) {
            emotion_[i] = emotion[i];
            cout<< emotion[i] << " ";
        }
        updateInit_emotion = true;
        cout<< endl ;   
    }

    void RobotStatusUpdate(double body[8] ){ 
        printf(  YELLOW "     Update Robot Status: "NONE);
        for(int i = 0 ; i < 8 ; i ++ ){
            body_[i] = body[i];
            cout<<  body[i] << " ";
        } 
        updateInit_status = true;
        cout<< endl ; 
    }
    
    void input_update(social_msg::perception_msg per ,double emotion[8], double body[8] ){ 
        
        for(int i = 0 ; i < 8 ; i ++ ) emotion_[i] = emotion[i];
        for(int i = 0 ; i < 8 ; i ++ ) body_[i] = body[i];
    }

    void perceptionClear(){
        per_list.clear();
    }
    
    bool updateInit(){  return (/* updateInit_perception &&  */ updateInit_emotion  /*&& updateInit_status*/ );}
    
    std::vector<need> need_compute_all(){
        cout<< "Start to Need Computation !!\n";
        std::vector<need> output_need_list;
        std::vector<need> temp;
        
        // a）当不存在外界感知信息时,机器人根据自身的情感和身体状态,生成内在需求,包括散步、闲聊、充电。
        if( per_list.size() == 0  ){   //即便感知为空,还是要把情感状态和身体状态的发送给  需求计算模型。
            social_msg::perception_msg per_none;  
            per_none.p = 0;
            task_model -> update( per_none, emotion_, body_ );  
            inner_model -> update(per_none, emotion_, body_); 
            
            // 任务性需求
            temp = task_model -> need_output();
            for(int j = 0 ; j < temp.size() ; j ++ )  output_need_list.push_back( temp[j] );
            // 生理性需求
            temp = inner_model -> need_compute_and_output();;
            for(int k = 0 ; k < temp.size() ; k ++ )  output_need_list.push_back( temp[k] );  
            // 这种情况下,不存在应急性需求
            
        }
       
        else{
            for( int i = 0 ; i < per_list.size(); i++ )
            {   
                // b）当存在外界感知信息且意图感知信息的概率方差,大于0.0025时（两概率之差0.1）,根据概率最高的意图生成需求。
                // b）当存在外界感知信息且意图感知信息的概率方差,大于0.000625时（两概率之差0.05）,根据概率最高的意图生成需求。
                social_msg::perception_msg per = per_list[i];
                if( varianceTop2(per) > 0.0006251 ) {
                    task_model -> update( per, emotion_, body_ );
                    inner_model -> update(per, emotion_, body_); //此时,也会有内部需求生成。  
                    emergency_model -> update( per );

                    // 任务性需求
                    temp = task_model -> need_output();
                    for(int j = 0 ; j < temp.size() ; j ++ )  output_need_list.push_back( temp[j] );
                    // 生理性需求
                    temp = inner_model -> need_compute_and_output();;
                    for(int k = 0 ; k < temp.size() ; k ++ )  output_need_list.push_back( temp[k] );  
                    // 应急性需求
                    temp = emergency_model -> need_output();
                    for(int k = 0 ; k < temp.size() ; k ++ )  output_need_list.push_back( temp[k] );  
                }
// 测试要求: 
//c）当存在外界感知信息且意图感知信息的概率方差,小于0.000625时（两概率之差0.02）,,若存在对应的场景先验信息,应该根据场景先验信息选择意图以生成需求。
//d）当存在外界感知信息且意图感知信息的概率方差,小于0.000625时,若不存在对应的场景先验信息,生成疑问需求。

//实际实现:  
// 方差小于0.000625时（两概率之差0.02）,且大于0.0001时（两概率之差0.02）,优先按照先验生成   【这部分 先验不一样,】
// 方差小于0.0001时（两概率之差0.02）, 一定生成doubt  。【这部分和测试要求的区别在于 ,   如果存在先验信息,那么一定是错误的,因此测试要求生成先验max】

//偏差结果: 
// 以测试要求  为正确结果。那么 我现在的程序,
// 在“方差小于0.0001时（两概率之差0.02）, 一定生成doubt” 这部分,如果存在先验信息,那么相对于“测试要求”一定是错误的。


                else if(  varianceTop2(per) >= 0.00010000  ){
                    //有哪些先验
                    if( per.intention == "EnterSchool" ||  per.intention_2 == "EnterSchool" ){
                        per.intention = "EnterSchool";
                    }  //测试要求认为EnterSchool和Uncooperat都很重要。但实际程序认为EnterSchool更重要。
                    else if( per.intention == "Uncooperate" ||  per.intention_2 == "Uncooperate" ){
                        per.intention = "Uncooperate";
                    }
                    else if(per.intention == "EnterHospital" ||  per.intention_2 == "EnterHospital"){
                        per.intention = "EnterHospital";
                    }
                    else if( (per.intention == "ScheduledDialysis"  && per.intention_2 == "UnscheduledDialysis") 
                          ||  (per.intention_2 == "ScheduledDialysis"  && per.intention == "UnscheduledDialysis") ){
                        per.intention = "UnscheduledDialysis";
                    }
                    else if(  per.intention == "ReturnHome" ||  per.intention_2 == "ReturnHome"  ){
                        per.intention = "ReturnHome";
                    }
                    else{ //没有先验了,生成doubt需求
                        inner_model -> update(per, emotion_, body_ , true);
                        // 生理性需求
                        temp = inner_model -> need_compute_and_output();;
                        for(int k = 0 ; k < temp.size() ; k ++ )  output_need_list.push_back( temp[k] );  
                        break; 
                    }
                    task_model -> update( per, emotion_, body_ );
                    inner_model -> update(per, emotion_, body_); 
                    emergency_model -> update( per );

                    // 任务性需求
                    temp = task_model -> need_output();
                    for(int j = 0 ; j < temp.size() ; j ++ )  output_need_list.push_back( temp[j] );
                    // 生理性需求
                    temp = inner_model -> need_compute_and_output();;
                    for(int k = 0 ; k < temp.size() ; k ++ )  output_need_list.push_back( temp[k] );  
                    // 应急性需求
                    temp = emergency_model -> need_output();
                    for(int k = 0 ; k < temp.size() ; k ++ )  output_need_list.push_back( temp[k] );  
                }

                else {   //这个时候一定要生成doubt
                    inner_model -> update(per, emotion_, body_ , true);
                    // 生理性需求
                    temp = inner_model -> need_compute_and_output();;
                    for(int k = 0 ; k < temp.size() ; k ++ )  output_need_list.push_back( temp[k] );  
                    break; 
                }
            
            }
        }

        //清空 perception
        perceptionClear();  //TODO: ?? 
        
        // printf 当前周期中生成的需求
        if( output_need_list.size() != 0 )
            for(int i = 0; i< output_need_list.size() ; i++  )  
            {
                std::cout <<  "    Output Need " << i+1 << ": " << output_need_list[i].need_name << " ,Weight: " <<output_need_list[i].weight;
                if (  output_need_list[i].person_name != "")
                    std::cout <<" ,for " <<output_need_list[i].person_name<<" as " <<output_need_list[i].IDtype;
                std::cout<<std::endl;
            }     
        else std::cout<<"    No need generated !\n";
        // 输出
        return output_need_list;
    }


};
#endif //TEST_CHECKRULE_H


    