#include "BehaviorManager.h"

// Read behavior data from .json file and save in behavior_library.
void BehaviorManager::readinBehaviorLibrary(const string &config_file)
{
    cout << "Start to read behavior data." << endl;
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
        return;
    }
    int behavior_num = root["behavior"].size();
    for(int i=0; i<behavior_num; i++){
        auto root_i = root["behavior"][i];
        Behavior behavior;
        string behavior_name = root_i["name"].asString();
        behavior_catalog.insert(behavior_name);
        
        behavior.name = behavior_name;
        behavior.weight = root_i["weight"].asDouble();
        behavior.type = root_i["type"].asString();
        behavior.is_light = root_i["is_light"].asBool();
        
        auto root_sub = root_i["subBehavior"];
        int sub_behavior_num = root_sub.size();
        vector<SubBehavior> sub_behaviors;

        for(int j=0; j<sub_behavior_num; j++){
            // readinArmConfig(root_sub[j],sub_behaviors);
            auto root_sub_j = root_sub[j];
            SubBehavior sub_behavior;
            sub_behavior.discription = root_sub_j["discription"].asString();

            // Gaze
            sub_behavior.gaze.is_necessary = root_sub_j["Gaze"]["is_necessary"].asBool();
            sub_behavior.gaze.weight = root_sub_j["Gaze"]["weight"].asDouble();
            // Emotion
            sub_behavior.emotion.is_necessary = root_sub_j["Emotion"]["is_necessary"].asBool();
            sub_behavior.emotion.weight = root_sub_j["Emotion"]["weight"].asDouble();
            // Voice
            sub_behavior.voice.is_necessary = root_sub_j["Voice"]["is_necessary"].asBool();
            sub_behavior.voice.weight = root_sub_j["Voice"]["weight"].asDouble();
            if (sub_behavior.voice.is_necessary) {sub_behavior.voice.content = root_sub_j["Voice"]["content"].asString();}
            // Manipulator
            sub_behavior.manipulator.is_necessary = root_sub_j["Manipulator"]["is_necessary"].asBool();
            sub_behavior.manipulator.weight = root_sub_j["Manipulator"]["weight"].asDouble();
            // Mover
            sub_behavior.mover.is_necessary = root_sub_j["Mover"]["is_necessary"].asBool();
            sub_behavior.mover.weight = root_sub_j["Mover"]["weight"].asDouble();

            sub_behaviors.push_back(sub_behavior);
        }
        
        behavior.subBehaviorSeries = sub_behaviors;
        behavior_library.insert(pair<string,Behavior>(behavior_name, behavior));
    }
}

// Handle need message and generate the behavior instance 
// with behavior_library data and need message's configuration.
bool BehaviorManager::readInNewNeed(const BehaviorModule::need_msg &msg)
{
    string need_name = msg.need_name;
    cout << "Get need : " << need_name << endl;

    // 1. Query the need in behavior_library.
    auto behavior_index = behavior_library.find(need_name);
    if (behavior_index == behavior_library.end()){
        return false;
    }
    Behavior new_behavior = behavior_index->second;

    // 2. Configure the behavior instance in different ways according to the type of behavior.
    if (new_behavior.type == "EXPRESSION"){}
    else if (new_behavior.type == "INTERACTION"){
        new_behavior.target = msg.person_name;
    }

    addNewBehavior(new_behavior);

    return true;
}

// Delete light behavior, add the new behavior instance into behaviorSeries
// and judge whether it is among the parallel behaviors to be execute.
void BehaviorManager::addNewBehavior(Behavior new_behavior)
{
    // Judge if behaviorSeries is empty.
    if(behaviorSeries.empty()){
        behaviorSeries.push_back(new_behavior);
        parallelNum = 1;
        updateBehaviorPub();
    }else{
        // Judge if a light behavior exists.
        if(behaviorSeries[0].is_light){
            behaviorSeries.clear();
            behaviorSeries.push_back(new_behavior);
            parallelNum = 1;
            updateBehaviorPub();
        }
    }
    
    // When behaviorSeries is not empty and there's no light behavior.
    int insertLocation = insertBehavior(new_behavior);
    parallelNum = computeParallel();
    if (insertLocation == parallelNum){
        updateBehaviorPub();
    }
    return;
}

// Make the necessary judge, update the parallel behaviors to be execute
// and finally pub them.
void BehaviorManager::updateBehaviorPub()
{
    if (behaviorSeries.empty()){
        return;
    }
    BehaviorModule::behavior_msg msg;
    msg.name = behaviorSeries[0].name;
    msg.type = behaviorSeries[0].type;
    publisher_behavior_.publish(msg);
    cout << "我正在执行：" << behaviorSeries[0].name << endl;

}

// Handle feedback from PerformModule:
// 1. Update progress of sub-behaviors into behaviorSeries and parallelBehaviorSeries.
// 2. Delete completed behaviors in behaviorSeries and parallelBehaviorSeries.
// 3. Start function updateBehaviorPub when necessary.
void BehaviorManager::behavior_feedback_callback(const BehaviorModule::behavior_feedback_msg &msg)
{
    cout << "I receive a bahavior feedback of : " << msg.hehavior_name << endl;
    cout << "It has been implemented up to step " << msg. behavior_phase << endl;
}

void BehaviorManager::printAllBehaviors()
{
    cout << "Behavior num : " << behavior_catalog.size() << "\n\n";
    cout << "Behavior names:\n";
    for(auto &name:behavior_catalog){
        cout << "   " << name << ",\n";
    }
    cout << "\n    'behavior':[\n";
    cout << "    {\n";
    for(auto &behavior_map:behavior_library){
        auto behavior = behavior_map.second;
        cout << "        {\n";
        cout << "            'name' : '" << behavior.name << "',\n";
        cout << "            'weight' : '" << behavior.weight << "',\n";
        cout << "            'type' : '" << behavior.type << "',\n";
        cout << "            'subBehavior':[\n";
        for (auto &sub_behavior:behavior.subBehaviorSeries){
            cout << "                {\n";
            cout << "                    'discription'   : '" << sub_behavior.discription << "',\n";
            cout << "                    'Gaze'          : {";
            if (sub_behavior.gaze.is_necessary) cout << "true";
            else cout << "false";
            cout << "," << sub_behavior.gaze.weight << "},\n";
            cout << "                    'Emotion'       : {";
            if (sub_behavior.emotion.is_necessary) cout << "true";
            else cout << "false";
            cout << "," << sub_behavior.emotion.weight << "},\n";
            cout << "                    'Voice'         : {";
            if (sub_behavior.voice.is_necessary) cout << "true";
            else cout << "false";
            cout << "," << sub_behavior.voice.weight << "},\n";
            cout << "                    'Manipulator'   : {";
            if (sub_behavior.manipulator.is_necessary) cout << "true";
            else cout << "false";
            cout << "," << sub_behavior.manipulator.weight << "},\n";
            cout << "                    'Mover'         : {";
            if (sub_behavior.mover.is_necessary) cout << "true";
            else cout << "false";
            cout << "," << sub_behavior.mover.weight << "},\n";
            
            cout << "                },\n";
        }
        cout << "        },\n";
    }
    cout << "    }\n";
}

int BehaviorManager::insertBehavior(Behavior &new_behavior)
{
    int index = 0;
    int num = behaviorSeries.size();
    double new_weight = new_behavior.weight;
    while(behaviorSeries[index].weight > new_weight && index < num ){
        i++;
    }

    if(index==num){
        behaviorSeries.push_back(new_behavior);
        return size;
    }
    else{
        behaviorSeries.push_back(behaviorSeries.back());
    }
    for(int i = num - 1 ; i > index ; i--){
        behaviorSeries[i] = behaviorSeries[i - 1];
    }
    behaviorSeries[index] = new_behavior;
    return index + 1;
}

int BehaviorManager::computeParallel()
{
    int parallel_num = 1;
    
    return parallel_num;
}