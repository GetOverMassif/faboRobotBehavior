/*
 * @Descripttion: 
 * @version: 
 * @Author: sueRimn
 * @Date: 2021-09-21 21:16:50
 * @LastEditors: Zhang Jiadong
 * @LastEditTime: 2023-05-09 10:40:02
 */
#include "task_paremeter_rw.h"

class task_need{

    public:
        std::vector<need> read_need_lists;
        social_msg::perception_msg per_;
        std::vector<double> rob_emotion_;
        std::string person_emotion_;
        std::vector<double> rob_status_;
        string yaml_name = "../../../personality_template/need/TaskNeed.yaml" ;
    public:
        task_need( ){
            /* 生成need */                
            read_need_parameter read;//(yaml_name);
            read_need_lists.clear();
            std::cout <<  "Read parameter for Task Need !! 类型如下:  " << "\n";
            std::string current_file_path = __FILE__;
            std::string current_folder_path = current_file_path.substr(0, current_file_path.find_last_of("/\\"));
            // std::cout << "Current folder path: " << current_folder_path << std::endl; 
            read_need_lists = read.return_task_need(current_folder_path + "/" + yaml_name );
        }
        
        void update( social_msg::perception_msg per, double *emotion, double body[8]){
                
                /* 读取need的判断信息 */
                per_ = per;
                // for(int i = 0 ; i < 8 ; i ++ )  rob_emotion_[i] = rob_emotion[i];   /* 这怎么错了？？ */
                std::vector<double > temp(emotion, emotion+8); rob_emotion_.assign(temp.begin(), temp.end());
                person_emotion_ = per.person_emotion;
                std::vector<double > temp2(body, body+8); rob_status_.assign(temp2.begin(), temp2.end());

            }

        std::vector<need> need_output(){
            std::vector<need>  output_need_lists = read_need_lists;
            for(std::vector<need>::iterator iter = output_need_lists.begin(); iter != output_need_lists.end();  ){
                if(
                    per_.intention ==  iter->intention  &&
                    per_.IDtype == iter->IDtype  &&
                    // per_.person_emotion_ == iter->person_emotion  &&
                    vector8_compare(rob_emotion_ , iter->rob_emotion)  &&                    
                    vector8_compare(rob_status_ , iter->rob_status ) 
                ){
                    iter->person_name = per_.person_name;
                    iter->target_angle = per_.target_angle;
                    iter->target_distance = per_.target_distance;
                    iter->speech = per_.speech;
                    iter->rob_emotion.assign(rob_emotion_.begin(), rob_emotion_.end());
                    robot_emotion_num2string( rob_emotion_ ,iter->robot_emotion_str ,iter->robot_emotion_intensity );
                    // iter->person_emotion.assign(person_emotion_.begin(), person_emotion_.end()); //用不到了。后续的行为模块只需要机器人情感的种类和强度
                    iter->person_emotion = person_emotion_;
                    iter->rob_status.assign(rob_status_.begin(), rob_status_.end()); 
                    iter ++;
                }
                else{
                    iter = output_need_lists.erase( iter ); 
                }
            }

            return output_need_lists;
        }
        bool vector8_compare(std::vector<double>& a, std::vector<double> &b){
            if(
                a[0] >= b[0]  &&   a[1] >= b[1]  &&   a[2] >= b[2]  &&    a[3] >= b[3]  &&
                a[4] >= b[4]  &&   a[5] >= b[5]  &&   a[6] >= b[6]  &&    a[7] >= b[7]  
            )
                return true;
            else
                return false;            
        }

        void wirte(){
            // std::ofstream fout(yaml_name);
            // doc[0]["name"] = i;
            // fout << doc;
            // fout << std::endl;fout << std::endl;
            // fout.close();
        }
        void weightUpdate(need_wu n){
            for(std::vector<need>::iterator iter = read_need_lists.begin(); iter != read_need_lists.end(); iter ++ )
                  if( n.need_name  ==  iter->need_name   &&     n.IDtype == iter->IDtype )
                    {
                        iter->weight +=  n.wu; 
                        printf("对[%s]的[%s]需求的权重更新[%f]",iter->IDtype, iter->need_name, n.wu);
                    }
            wirte();
        }
    private:
        

double emotion_calm_thresh = 0.3;
double emotion_intensity_thresh_3 = 0.8;
double emotion_intensity_thresh_2 = 0.5;
double emotion_intensity_thresh_1 = 0.3;


int robot_emotion_num2intensity( double& rob_emotion_value){
    if(rob_emotion_value>emotion_intensity_thresh_3)
        return 3;
    else if(rob_emotion_value>emotion_intensity_thresh_2)
        return 2;
    else if(rob_emotion_value>emotion_intensity_thresh_1)
        return 1;
    return 0;
}


void robot_emotion_num2string( std::vector<double> rob_emotion , string & rob_emotion_string , int & rob_emotion_intensity ){
    int temp  =  -1; 
    int i_EmotionMax;
    int i = 0;
    for(  ; i < rob_emotion.size() ; i++) {
        if( rob_emotion[i] > temp ){
            temp = rob_emotion[i];
            i_EmotionMax = i ;
        }
        // else if(rob_emotion[i] == temp){
        //     ROS_DEBUG("robot_emotion_num2string: 最大情绪值存在两个:  [%d] 和 [%d] ",i_EmotionMax,i);
        // }
    }
    
    if( rob_emotion[i_EmotionMax] < emotion_calm_thresh){
        rob_emotion_string =  "Calm";
        rob_emotion_intensity = 0;
    }
    else{
        rob_emotion_intensity = robot_emotion_num2intensity(rob_emotion[i_EmotionMax]);
        switch( i_EmotionMax )
        {
        case 0:
            rob_emotion_string =  "Joy"; 
            break;
        case 1:
            rob_emotion_string =  "Trust"; 
            break;
        case 2:
            rob_emotion_string =  "Suprise"; 
            break;
        case 3:
            rob_emotion_string =  "Sadness"; 
            break;
        case 4:
            rob_emotion_string =  "Anger"; 
            break;
        case 5:
            rob_emotion_string =  "Fear"; 
            break;
        case 6:
            rob_emotion_string =  "Disgust"; 
            break;
        case 7:
            rob_emotion_string =  "Boring"; 
            break;
        }
    }
}
        
};