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
        behavior.total_phase = behavior.subBehaviorSeries.size();
        behavior_library.insert(pair<string,Behavior>(behavior_name, behavior));
    }
}

// Handle need message and generate the behavior instance 
// with behavior_library data and need message's configuration.
bool BehaviorManager::readInNewNeed(const BehaviorModule::need_msg &msg)
{
    string need_name = msg.need_name;
    cout << "【addNewBehavior】" << need_name;

    // 1. Query the need in behavior_library.
    auto behavior_index = behavior_library.find(need_name);
    if (behavior_index == behavior_library.end()){
        cout << " fails." << endl;
        return false;
    }
    cout << " succeeds." << endl;

    Behavior new_behavior(behavior_index->second);

    new_behavior.configureByNeedMsg(msg);

    // TODO: Configure behavior

    // 2. Configure the behavior instance in different ways according to the type of behavior.
    // if (new_behavior.type == "EXPRESSION"){}
    // else if (new_behavior.type == "INTERACTION"){
    //     new_behavior.target = msg.person_name;
    // }
    

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
            printCurrentSeries();
            updateBehaviorPub();
            return;
        }
    }
    
    // When behaviorSeries is not empty and there's no light behavior.
    int insertLocation = insertBehavior(new_behavior);
    parallelNum = computeParallel();
    printCurrentSeries();
    // cout << "insertLocation = " << insertLocation << endl; 
    // cout << "parallelNum = " << parallelNum << endl; 
    if (insertLocation <= parallelNum){
        updateBehaviorPub();
    }
    return;
}

void BehaviorManager::tellIdleState()
{
    //TODO: tell EmotionModule the idle state
    // to be considered: 从空闲状态转换到有行为执行时是否需要告知情绪模块
}

// Make the necessary judge, update the parallel behaviors to be execute
// and finally pub them.
void BehaviorManager::updateBehaviorPub()
{
    if (behaviorSeries.empty()){
        return;
    }

    BehaviorModule::behavior_msg msg;

    if (!mvCurrentBehaviors.empty()){
        if(!pauseFlag)
        {
            msg.name = "Pause";
            publisher_behavior_.publish(msg);
            printMsgInfo(msg);
            pauseFlag = true;
            cout << "Set pauseFlag true.\n\n";
        }
        return;
    }

    vector<BehaviorModule::behavior_msg> msgs;

    for(int i = 0 ; i < parallelNum ; i++){
        mvCurrentBehaviors.push_back(behaviorSeries[i]);
        msg = generateOrderMsgByBehavior(behaviorSeries[i]);

        for(int j = 0 ; j < 5 ; j ++) {
            msg.occupancy[j] = 0;
        }
        msgs.push_back(msg);
    }
    // cout << "occupancy : {" << occupancy[0];
    // for(int i = 1 ; i < 5 ; i++) {cout << "," << occupancy[i];}
    // cout << "}" << endl;

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
    cout << "【BehaviorFeedback】" << msg.hehavior_name << "," << (int)msg.current_phase << endl;

    // to be tested: varify behavior by stamp
    bool rightBehaviorFlag = false;
    vector<Behavior>::iterator itor = behaviorSeries.begin();

    for (auto &behavior : behaviorSeries){
        if(judgeSameStamp(behavior.header, msg.header)){
            rightBehaviorFlag = true;
            if(msg.current_phase > behavior.current_phase && 
                            msg.current_phase <= behavior.total_phase)
            {
                behavior.current_phase = msg.current_phase;
                if (msg.current_phase == behavior.total_phase)
                {
                    behaviorSeries.erase(itor);
                }
                printCurrentSeries();
                break;
            }
            else
            {
                cout << "Right behavior, wrong phase." << endl;
                return;
            }
        }
        itor++;
    }

    if (!rightBehaviorFlag){
        cout << "No behavior \"" << msg.hehavior_name << "\" in the whole behavior series." << endl;
        return;
    }

    rightBehaviorFlag = false;
    itor = mvCurrentBehaviors.begin();

    for (auto &behavior : mvCurrentBehaviors){
        if(judgeSameStamp(behavior.header, msg.header)){

            rightBehaviorFlag = false;
            behavior.current_phase = msg.current_phase;

            if (msg.current_phase == behavior.total_phase || pauseFlag){
                // finish one behavior
                mvCurrentBehaviors.erase(itor);
                if(mvCurrentBehaviors.empty() && !behaviorSeries.empty()){
                    if(!pauseFlag) parallelNum = 1;
                    occupancy = {1,1,1,1,1};
                    pauseFlag = false;
                    updateBehaviorPub();
                }
                else if (behaviorSeries.empty()) {
                    tellIdleState();
                }
            }
            printCurrentSeries();
            return;
        }
        itor++;
    }
    cout << "No behavior \"" << msg.hehavior_name << "\" in current behavior series to be paused." << endl;
    return;
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
        // printCurrentSeries();
        return 1;
    }
    double new_weight = new_behavior.weight;
    while(behaviorSeries[index].weight >= new_weight && index < num ){
        index++;
    }

    if(index==num){
        behaviorSeries.push_back(new_behavior);
        // printCurrentSeries();
        return index + 1;
    }
    else{
        behaviorSeries.push_back(behaviorSeries.back());
    }
    for(int i = num - 1 ; i > index ; i--){
        behaviorSeries[i] = behaviorSeries[i - 1];
    }
    behaviorSeries[index] = new_behavior;
    // printCurrentSeries();
    return index + 1;
}

int BehaviorManager::computeParallel()
{
    int parallel_num = 0;
    vector<int> count = {0,0,0,0,0};
    occupancy = vector<int>{1,1,1,1,1};
    vector<int> local_count;
    for (auto &behavior:behaviorSeries)
    {
        for(int i = 0 ; i < 5 ; i++){
            if(behavior.necessary_count[i]){
                local_count.push_back(i);
                if(count[i] == 1){
                    cout << "Compute parallel_num = " << parallel_num << endl;
                    return parallel_num;
                }
            }
        }
        parallel_num ++;
        for(auto &index:local_count) {
            count[index] ++;
            occupancy[index] = parallel_num;
        }
    }
    cout << "Compute parallel_num = " << parallel_num << endl;
    return parallel_num;
}

void BehaviorManager::printCurrentSeries()
{
    cout << "\n【printCurrentSeries】" << endl;
    if(behaviorSeries.empty()){
        cout << "BehaviorSeries is empty now." << endl;
        return;
    }
    cout << "Order\t" << "weight\t" << "phase\t" << "necessary\t" << "name\t" << endl;
    int order = 1;
    for(auto behavior:behaviorSeries){
        cout << order << "\t" 
             << behavior.weight << "\t"
             << behavior.current_phase << "/" << behavior.total_phase << "\t"
             << "{" << behavior.necessary_count[0] ;
        for(int i = 1 ; i < 5 ; i++){
            cout << "," << behavior.necessary_count[i];
        }
        cout << "} " << "\t" << behavior.name << "\t" << endl;
        order ++;
    }
    cout << "行为停止后有待并行的行为数量 :" << parallelNum << endl;
    cout << "当前正在并行的行为数量 : " << mvCurrentBehaviors.size() << endl;
    cout << endl;
    return;
}

void BehaviorManager::printMsgInfo(BehaviorModule::behavior_msg msg)
{
    cout << "【Sent behavior_msg】" << endl;
    cout << msg.name << "\t" << msg.type << "\t" << (int)msg.current_phase << "/" << (int)msg.total_phase << "\t";
    cout << "{" << (int)msg.occupancy[0];
    for(int i = 1 ; i < 5 ; i++){
        cout << "," << (int)msg.occupancy[i];
    }
    cout << "}" << endl << endl;
    return;
}