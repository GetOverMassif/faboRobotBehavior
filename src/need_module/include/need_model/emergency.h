/*
 * @Descripttion: 
 * @version: 
 * @Author: Zhang Jiadong
 * @Date: 2022-03-08 17:05:40
 * @LastEditors: Zhang Jiadong
 * @LastEditTime: 2023-05-09 10:37:06
 */

#include "common_include.h"

class Emergency {
    private:
        std::vector<need> read_need_lists;
        social_msg::perception_msg per_;
        string yaml_name = "../../../personality_template/need/Emergency.yaml" ;

    public:
        Emergency(){
            /* 生成need */                
            read_need_parameter read;//(yaml_name);
            read_need_lists.clear();
            std::cout <<  "Read parameter for Emergency Need !! 类型如下:  " << "\n"; 
            std::string current_file_path = __FILE__;
            std::string current_folder_path = current_file_path.substr(0, current_file_path.find_last_of("/\\"));
            // std::cout << "Current folder path: " << current_folder_path << std::endl;
            read_need_lists = read.return_emergency_need( current_folder_path + "/" + yaml_name );
        }

        void update( social_msg::perception_msg per ){
                
                /* 读取need的判断信息 */
                per_ = per;
            }


        std::vector<need> need_output(){
            std::vector<need>  output_need_lists = read_need_lists;
            for(std::vector<need>::iterator iter = output_need_lists.begin(); iter != output_need_lists.end();  ){
                if(
                    per_.intention ==  iter->intention  
                ){
                    iter->person_name = per_.person_name;
                    iter->target_angle = per_.target_angle;
                    iter->target_distance = per_.target_distance;
                    iter->speech = per_.speech;
                    iter->person_emotion = per_.person_emotion;
                    iter ++;
                }
                else{
                    iter = output_need_lists.erase( iter ); 
                }
            }

            return output_need_lists;
        }

        void wirte(){
            // std::ofstream fout(yaml_name);
            // doc[0]["name"] = i;
            // fout << doc;
            // fout << std::endl;fout << std::endl;
            // fout.close();
        }



};