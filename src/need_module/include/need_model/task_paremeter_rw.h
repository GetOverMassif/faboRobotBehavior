/*
 * @Descripttion: 
 * @version: 
 * @Author: sueRimn
 * @Date: 2021-09-21 21:16:50
 * @LastEditors: Zhang Jiadong
 * @LastEditTime: 2022-04-01 22:55:14
 */

#ifndef CONFIG_H
#define CONFIG_H

// std 
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>
#include "common_include.h"
using namespace std; 



class read_need_parameter
{
private:
    
    YAML::Node task_doc;
    YAML::Node emergency_doc;
    vector<need> need_lists;
public:
    read_need_parameter () {} // private constructor makes a singleton
    // read_need_parameter (const string DB_CONF ) {   //DB_CONF:  yaml文件地址
        
    // }

public:
    ~read_need_parameter(){};  // close the file when deconstructing 

    vector<need>  return_task_need(const string DB_CONF){
        task_doc = YAML::LoadFile( DB_CONF );
        cout << "从cfg文件中的need数量:" << task_doc.size() << endl;
        std::cout << "flag2" << std::endl;
        for(unsigned int i=0; i<task_doc.size(); i++) {
            std::cout << "flag4" << std::endl;
            need temp;
            std::cout << task_doc[i]["need_name"] << std::endl;
            task_doc[i]["need_name"] >> temp.need_name;
            std::cout << "need_name" << std::endl;
            task_doc[i]["intention"]  >> temp.intention ;
            std::cout << "intention" << std::endl;
            task_doc[i]["IDtype"]  >> temp.IDtype ; 
            std::cout <<  "    第" << i+1 << "个: " << temp.need_name << "\n";
            task_doc[i]["person_emotion"]  >> temp.person_emotion ;
            task_doc[i]["rob_emotion"] >> temp.rob_emotion;
            task_doc[i]["rob_status"] >> temp.rob_status;
            task_doc[i]["weight"]  >> temp.weight ;
            task_doc[i]["satisfy_value"]  >> temp.satisfy_value;
            temp.speech = "" ;
            std::cout << "flag3" << std::endl;
            need_lists.push_back(temp);
        }
        std::cout << "flag2" << std::endl;
        return need_lists;
    }

    vector<need>  return_emergency_need(const string DB_CONF){
         emergency_doc = YAML::LoadFile( DB_CONF );
         cout<<"从cfg文件中的need数量:"<<emergency_doc.size()<<endl;
         for(unsigned i=0; i<emergency_doc.size(); i++) {
            need temp;
            emergency_doc[i]["need_name"]  >> temp.need_name ;                   
            emergency_doc[i]["intention"]  >> temp.intention ;
            std::cout <<  "    第" << i+1 << "个: " << temp.need_name << "\n";
            emergency_doc[i]["weight"]  >> temp.weight ;
            temp.speech = "" ;

            need_lists.push_back(temp);
        }
        return need_lists;
    }
};


#endif // CONFIG_H