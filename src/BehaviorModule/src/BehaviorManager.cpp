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

            behavior.necessary_count[0] += sub_behavior.gaze.is_necessary?1:0;
            behavior.necessary_count[1] += sub_behavior.emotion.is_necessary?1:0;
            behavior.necessary_count[2] += sub_behavior.voice.is_necessary?1:0;
            behavior.necessary_count[3] += sub_behavior.manipulator.is_necessary?1:0;
            behavior.necessary_count[4] += sub_behavior.mover.is_necessary?1:0;
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
    // cout << "Get need : " << need_name << endl;

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
    if(!behaviorSeries.empty()){
        // Judge if a light behavior exists.
        if(behaviorSeries[0].is_light){
            behaviorSeries.clear();
            insertBehavior(new_behavior);
            parallelNum = 1;
            updateBehaviorPub();
            return;
        }
    }
    
    // When behaviorSeries is not empty and there's no light behavior.
    int insertLocation = insertBehavior(new_behavior);
    parallelNum = computeParallel();
    cout << "insertLocation = " << insertLocation << endl; 
    cout << "parallelNum = " << parallelNum << endl; 
    if (insertLocation <= parallelNum){
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
    if (!parallelBehaviorSeries.empty()){
        msg.name = "Stop";
        publisher_behavior_.publish(msg);
        printMsgInfo(msg);
    }
    vector<BehaviorModule::behavior_msg> msgs;
    for(int i = 0 ; i < parallelNum ; i++){
        msg.name = behaviorSeries[i].name;
        msg.type = behaviorSeries[i].type;
        msg.behavior_phase = behaviorSeries[i].behavior_phase;
        // msg.occupancy = new int8[5];
        for(int j = 0 ; j < 5 ; j++) {msg.occupancy[j] = 1;}
        msgs.push_back(msg);
    }

    for(int i = 0 ; i < 5 ; i++){
        msgs[occupancy[i]-1].occupancy[i] = 1;
    }

    for(auto &one_msg:msgs)
    {
        publisher_behavior_.publish(one_msg);
        printMsgInfo(one_msg);
    }
    return;
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
    if(!num){
        behaviorSeries.push_back(new_behavior);
        printCurrentSeries();
        return 1;
    }
    double new_weight = new_behavior.weight;
    while(behaviorSeries[index].weight >= new_weight && index < num ){
        index++;
    }

    if(index==num){
        behaviorSeries.push_back(new_behavior);
        printCurrentSeries();
        return index + 1;
    }
    else{
        behaviorSeries.push_back(behaviorSeries.back());
    }
    for(int i = num - 1 ; i > index ; i--){
        behaviorSeries[i] = behaviorSeries[i - 1];
    }
    behaviorSeries[index] = new_behavior;
    printCurrentSeries();
    return index + 1;
}

int BehaviorManager::computeParallel()
{
    int parallel_num = 0;
    vector<int> count = {0,0,0,0,0};
    occupancy = vector<int>{1,1,1,1,1};
    for (auto &behavior:behaviorSeries)
    {
        for(int i = 0 ; i < 5 ; i++){
            if(behavior.necessary_count[i]){
                count[i] += 1;
                occupancy[i] = parallel_num;
            }
            if(count[i]>1){
                return parallel_num;
            }
        }
        parallel_num ++;
    }
    return parallel_num;
}

void BehaviorManager::printCurrentSeries()
{
    if(behaviorSeries.empty()){
        cout << "BehaviorSeries is empty now." << endl;
        return;
    }
    cout << endl;
    cout << "Order\t" << "weight\t" << "num\t" << "necessary\t" << "name\t" << endl;
    int order = 1;
    for(auto behavior:behaviorSeries){
        cout << order << "\t" 
             << behavior.weight << "\t"
             << behavior.subBehaviorSeries.size() << "\t"
             << "{" << behavior.necessary_count[0] ;
        for(int i = 1 ; i < 5 ; i++){
            cout << "," << behavior.necessary_count[i];
        }
        cout << "} " << "\t" << behavior.name << "\t" << endl;
        order ++;
    }
    cout << endl;
    return;
}

void BehaviorManager::printMsgInfo(BehaviorModule::behavior_msg msg)
{
    cout << msg.name << "\t" << msg.type << "\t" << msg.behavior_phase << "\t";
    cout << "{" << msg.occupancy[0];
    for(int i = 1 ; i < 5 ; i++){
        cout << "," << msg.occupancy[i];
    }
    cout << "}" << endl;
    return;
}