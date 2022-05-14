#include "ArmsActionManager.h"

void ArmsActionManager::readinActions(const string &config_file){
    cout << "start to read data." << endl;
    Json::Value root;
    Json::Reader reader;
    fstream ifs(config_file);

    string strjson;
    if (!ifs.is_open()) {
        cout << "Fail to open ifs." << endl;
        return;
    }

    string strline;
    while (getline(ifs, strline)) {
        strjson += strline;
    }
    ifs.close();

    if (!reader.parse(strjson, root)){
		cout << "Fail to open json." << endl;
		ifs.close();
	}
    else{
        int arm_action_num = root["arm_action"].size();
        // 分别添加 arm_action_num 个不同动作
        for(int i=0; i<arm_action_num; i++){
            auto root_i = root["arm_action"][i];
            string action_name = root_i["name"].asString();
            auto root_i_l = root_i["left_arm_action"];
            auto root_i_r = root_i["right_arm_action"];
            int left_arm_action_num = root_i_l.size();
            int right_arm_action_num = root_i_r.size();

            action_catalog.insert(action_name);
            vector<ArmConfig> left_arm_action,right_arm_action;

            for(int j=0; j<left_arm_action_num; j++){
                readinArmConfig(root_i_l[j],left_arm_action);
            }

            for(int j=0; j<right_arm_action_num; j++){
                readinArmConfig(root_i_r[j],right_arm_action);
            }

            action_library.insert(pair<string,ArmsAction>(action_name,ArmsAction(action_name,left_arm_action,right_arm_action)));
        }
    }
}

void ArmsActionManager::readinArmConfig(Json::Value root,std::vector<ArmConfig>& single_arm_action){
    bool keepStill = root["keep_still"].asBool();
    double jointPos[6];
    for(int k=0; k<6; k++){
        jointPos[k] = root["jointPos"][k].asDouble();
    }
    double speed = root["speed"].asDouble();
    double pausetime = root["pausetime"].asDouble();
    single_arm_action.push_back(ArmConfig(keepStill,jointPos,speed,pausetime));
}

void ArmsActionManager::printAllActions(){
    cout << "Action num : " << action_catalog.size() << "\n\n";
    cout << "Action names:\n";
    for(auto &name:action_catalog){
        cout << "   " << name << ","<< endl;
    }
    cout << endl;
    cout << "\"arm_action\":[";
    for(auto &arm_action:action_library){
        cout << "\n    {\n        \"name\":\"" << arm_action.first << "\"," << endl;
        // left
        cout << "        \"left_arm_action\":[";
        for(auto &arm_config:arm_action.second.left_arm_action){
            cout << "\n            {";
            cout << "\"keep_still\": " << arm_config.keepStill << ", ";
            cout << "\"jointPos\":[";
            for(int i=0;i<6;i++){
                cout << " " << arm_config.jointPos[i] << ",";
            }
            cout << "] \"speed\": " << arm_config.speed << ",}";
        }
        cout << "\n        ],\n    }\n";
        // right
        cout << "        \"right_arm_action\":[";
        for(auto &arm_config:arm_action.second.right_arm_action){
            cout << "\n            {";
            cout << "\"keep_still\": " << arm_config.keepStill << ", ";
            cout << "\"jointPos\":[";
            for(int i=0;i<6;i++){
                cout << " " << arm_config.jointPos[i] << ",";
            }
            cout << "] \"speed\": " << arm_config.speed << ",}";
        }
        cout << "\n        ],\n    }\n";
        
    }
}
