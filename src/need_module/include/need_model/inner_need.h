#include "common_include.h"

class inner_need{

    public:
        /* 上一次机器人处于工作状态的时间 */
        // // static 
        // static time_t time_for_wandor;
        time_t time_lastSpareStates;
        bool Init = true;
        /* need的判断信息 */
        social_msg::perception_msg per_;
        string IDtype_ = "itself";
        std::vector<double> rob_emotion_;
        std::string person_emotion_;
        std::vector<double> rob_status_;
        std::vector<double> rob_status_last;

        bool doubtflag_;
        /* inner的need的影响因子 */
        double  Doubt_factor ;      double  Doubt_weight = 0.95;
        double  Wander_factor ;     double  Wander_weight = 0.8;    double  Wander_time_thresh = 10;
        double  Chat_factor ;       double  Chat_weight = 0.8;
        double  Charge_factor ;     double  Charge_weight = 1;


        
    public:
        inner_need( ){
            /* 生成need */
            std::string current_file_path = __FILE__;
            std::string current_folder_path = current_file_path.substr(0, current_file_path.find_last_of("/\\"));
            // std::cout << "Current folder path: " << current_folder_path << std::endl;
            string yaml_name = "../../../personality_template/need/InnerNeed.yaml" ;
            YAML::Node inner_doc = YAML::LoadFile( current_folder_path + "/" + yaml_name  );
            std::cout <<  "Read parameter for Inner Need !! 类型如下:  " << "\n"; 
            inner_doc["Doubt_factor"] >> Doubt_factor ;
            inner_doc["Doubt_weight"] >> Doubt_weight ;
            std::cout <<  "    第1个: Doubt \n";
            inner_doc["Wander_factor"] >> Wander_factor;
            inner_doc["Wander_weight"] >> Wander_weight;
            std::cout <<  "    第2个: Wander \n";
            inner_doc["Chat_factor"] >> Chat_factor;
            inner_doc["Chat_weight"] >> Chat_weight;
            std::cout <<  "    第3个: Chat \n";
            inner_doc["Charge_factor"] >> Charge_factor;
            inner_doc["Charge_weight"] >> Charge_weight;
            std::cout <<  "    第4个: Charge \n";
            /* time wandor */
            time_lastSpareStates = 0;
            // rob_status_ = {1,1,1,1,1,1,1,1};
        }

        ~inner_need(){};

        void update( social_msg::perception_msg per, double *emotion, double *body , bool doubtflag = false){
                /* 读取need的判断信息 */
                per_ = per;
                doubtflag_ = doubtflag;
                /* 更新外界状态 */
                std::vector<double > temp(emotion, emotion+8); rob_emotion_.assign(temp.begin(), temp.end());
                person_emotion_ = per.person_emotion;
                rob_status_last.assign(rob_status_.begin(), rob_status_.end());
                std::vector<double > temp2(body, body+8); rob_status_.assign(temp2.begin(), temp2.end());

                /* 更新 time */
                if(Init)  {time_lastSpareStates = time(NULL);  Init = false;}
                else if(rob_status_[7] == 1  && rob_status_last[7] == 0 )  
                    time_lastSpareStates = time(NULL);
            }
    private:
        need Doubt( ){
            need temp;
            temp.need_name = "Doubt";

            temp.intention = "";
            temp.IDtype = IDtype_; 
            temp.target_angle = 0.0;
            temp.target_distance = 0.0;
            temp.rob_emotion.assign(rob_emotion_.begin(), rob_emotion_.end());
            robot_emotion_num2string( rob_emotion_ ,temp.robot_emotion_str ,temp.robot_emotion_intensity );
            temp.person_name = "itself";
            temp.IDtype = "itself";
            temp.person_emotion = "none";            
            temp.rob_status.assign(rob_status_.begin(), rob_status_.end());
            // temp.weight = Doubt_factor * (1 - Doubt_weight) * (1 - p_)/0.5 + Doubt_weight;   
            // if(p_ > 0) 
            //     {temp.weight = Doubt_weight * (-1*pow(Doubt_factor , (1-p_)*10) + 1);  }
            
            if(doubtflag_){
                temp.weight = Doubt_weight;  //TODO: ??
                doubtflag_ = false;
            }
                
            // temp.speech = speech_;
            temp.speech = "请问您需要我做什么吗";
            temp.satisfy_value = 2 ;
            return temp;
        }

        need Wander( ){
            need temp;
            /* 信息 */
            temp.need_name = "Wander";
            /* 评价标准 */
            temp.intention = "";
            temp.IDtype = ""; 
            temp.target_angle = 0.0;
            temp.target_distance = 0.0;
            temp.rob_emotion.assign(rob_emotion_.begin(), rob_emotion_.end());
            robot_emotion_num2string( rob_emotion_ ,temp.robot_emotion_str ,temp.robot_emotion_intensity );
            temp.person_name = "itself";
            temp.IDtype = "itself";
            temp.person_emotion = "none";            
            temp.rob_status.assign(rob_status_.begin(), rob_status_.end());
            /* 权重计算公式 */
            /* 更新 time */
            // if(rob_status_[7] == 1) {
            //     time_t now = time(NULL);
            //     printf("[For Debug] 闲置时间为: %f秒\n",difftime(now, time_for_wandor));
            //     temp.weight = /* Wander_factor * */ (1 - Wander_weight) * (difftime(now, time_for_wandor) - Wander_time_thresh)  + Wander_weight;   
            
            // }
            // else{
            //     temp.weight =  Wander_weight;
            // }
            if(rob_status_[7] == 1)  {   
                time_t timeNow = time(NULL);
                temp.weight = Wander_weight * (-1*pow(Wander_factor , difftime(timeNow, time_lastSpareStates)/60) + 1);
            }
            else  temp.weight = 0;
            /* 语音内容 */
            // temp.speech = speech_;
            temp.speech = "怎么还没有人来";
            temp.satisfy_value = 2 ;
            return temp;
            }
        need Wander_boring(){
            need temp;
            /* 信息 */
            temp.need_name = "Wander";
            /* 评价标准 */
            temp.intention = "";
            temp.IDtype = ""; 
            temp.target_angle = 0.0;
            temp.target_distance = 0.0;
            temp.rob_emotion.assign(rob_emotion_.begin(), rob_emotion_.end());
            robot_emotion_num2string( rob_emotion_ ,temp.robot_emotion_str ,temp.robot_emotion_intensity );
            temp.person_name = "itself";
            temp.IDtype = "itself";
            temp.person_emotion = "none";            
            temp.rob_status.assign(rob_status_.begin(), rob_status_.end());
            /* 权重计算公式 */
            // temp.weight = Chat_factor * (1 - Chat_weight) * rob_emotion_[3] + Chat_weight; // TODO: 第几个是"无聊"情绪。假设为: 高兴、悲伤、愤怒、无聊
            temp.weight = Chat_weight * (-1*pow(Chat_factor , rob_emotion_[7]*10) + 1);
            /* 语音内容 */
            // temp.speech = speech_;
            temp.speech = "怎么还没有人来";
            temp.satisfy_value = 2 ;
            return temp;
            }

        need Chat( ){
            need temp;
            /* 信息 */
            temp.need_name = "Chat";
            /* 评价标准 */
            temp.intention = "";
            temp.IDtype = IDtype_; 
            temp.target_angle = 0.0;
            temp.target_distance = 0.0;
            temp.rob_emotion.assign(rob_emotion_.begin(), rob_emotion_.end());
            robot_emotion_num2string( rob_emotion_ ,temp.robot_emotion_str ,temp.robot_emotion_intensity );
            temp.person_name = "itself";
            temp.IDtype = "itself";
            temp.person_emotion = "none";            
            temp.rob_status.assign(rob_status_.begin(), rob_status_.end());
            /* 权重计算公式 */
            // temp.weight = Chat_factor * (1 - Chat_weight) * rob_emotion_[3] + Chat_weight; // TODO: 第几个是"无聊"情绪。假设为: 高兴、悲伤、愤怒、无聊
            temp.weight = Chat_weight * (-1*pow(Chat_factor , rob_emotion_[7]*10) + 1);
            /* 语音内容 */
            // temp.speech = speech_;
            temp.speech = "你来和我聊聊天吧";
            temp.satisfy_value = 2 ;
            return temp;
            }

        need Charge( ){
            need temp;
            /* 信息 */
            temp.need_name = "Charge";
            /* 评价标准 */
            temp.intention = "";
            temp.IDtype = ""; 
            temp.target_angle = 0.0;
            temp.target_distance = 0.0;
            temp.rob_emotion.assign(rob_emotion_.begin(), rob_emotion_.end());
            robot_emotion_num2string( rob_emotion_ ,temp.robot_emotion_str ,temp.robot_emotion_intensity );
            temp.person_name = "itself";
            temp.IDtype = "itself";
            temp.person_emotion = "none";            
            temp.rob_status.assign(rob_status_.begin(), rob_status_.end());
            /* 权重计算公式 */
            // temp.weight = Charge_factor * (1 - Charge_weight) * (1 - rob_status_[0]) + Charge_weight; 
            if(rob_status_[0] > Charge_factor)  temp.weight = 0;
            else temp.weight = Charge_weight * (1 - Charge_factor * rob_status_[0]) ;// TODO: 第几个 电量
            /* 语音内容 */
            // temp.speech = speech_;
            temp.speech = "没电了 没电了";
            temp.satisfy_value = 2 ;
            return temp;
            }
    public:
        vector<need> need_compute_and_output(){
            std::vector<need> output_need_lists;
            need temp;
            temp = Doubt();  if( temp.weight > 0.5 )    output_need_lists.push_back(temp);
            temp = Wander_boring();  if( temp.weight > 0.5 )    output_need_lists.push_back(temp);
            // temp = Wander();  if( temp.weight > 0.5 )   output_need_lists.push_back(temp);/* 应该让漫步的需求,永久输出？ */
            // temp = Chat();   if( temp.weight > 0.5 )    output_need_lists.push_back(temp);
            temp = Charge(); if( temp.weight > 0.5 )    output_need_lists.push_back(temp);

            return output_need_lists;
        }

        void wirte(){
            // std::ofstream fout(yaml_name);
            // doc[0]["name"] = i;
            // fout << doc;
            // fout << std::endl;fout << std::endl;
            // fout.close();
        }

        void weightUpdate(need_wu n){
                if( n.need_name  ==  "Doubt" ){
                        Doubt_factor += n.wu;
                        printf("[Doubt]需求的权重更新[%f]", n.wu);
                }
                if(  n.need_name == "Chat" ){
                        Chat_factor += n.wu;
                        printf("[Chat]需求的权重更新[%f]",  n.wu);
                }
                if( n.need_name == "Charge" ){
                        Charge_factor += n.wu;
                        printf("[Charge]需求的权重更新[%f]",  n.wu);
                }
                if( n.need_name == "Wander" ){
                        Wander_factor += n.wu;
                        printf("[Wander]需求的权重更新[%f]", n.wu);
                }

                wirte();
        }

private:   


double emotion_calm_thresh = 0.3;
double emotion_intensity_thresh_3 = 0.8;
double emotion_intensity_thresh_2 = 0.5;
double emotion_intensity_thresh_1 = 0.3;


int robot_emotion_num2intensity(double& rob_emotion_value){
    if (rob_emotion_value > emotion_intensity_thresh_3)
        return 3;
    else if (rob_emotion_value > emotion_intensity_thresh_2)
        return 2;
    else if (rob_emotion_value > emotion_intensity_thresh_1)
        return 1;
    return 0;
}

void robot_emotion_num2string(std::vector<double> rob_emotion, string & rob_emotion_string, int & rob_emotion_intensity ){
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
    
    if (rob_emotion[i_EmotionMax] < emotion_calm_thresh){
        rob_emotion_string = "Calm";
        rob_emotion_intensity = 0;
    }
    else {
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


// // for(int i = 0 ; i < 8 ; i ++ ) temp.rob_emotion[i] = rob_emotion_[i];
//             for(int i = 0 ; i < 8 ; i ++ ) temp.person_emotion[i] = person_emotion_[i];
//             for(int i = 0 ; i < 8 ; i ++ ) temp.rob_status[i] = rob_status_[i];