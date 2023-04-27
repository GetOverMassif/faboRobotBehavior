# 可能需要发送的 topic : need, behavior_feedback

import argparse
import os

if __name__=="__main__":
    parser = argparse.ArgumentParser(description="Hello")
    parser.add_argument("--topic", type=str, default="need")
    parser.add_argument("--action", type=str, default="wave")
    parser.add_argument("--need", type=str, default="Greet")
    parser.add_argument("--feedname", type=str, default="Greet")
    parser.add_argument("--time", type=int, default=0)

    args = parser.parse_args()

    s1 = "{"
    s2 = "}"

    order_arm = f'rostopic pub -1 /arm_action arm_control/Arms "{s1}call: false, weight: 0, action: \'{args.action}\', rate: 0, angle: 0, startTime: 0, endTime: 0, IsYellow: false{s2}"'
    order_need = f'rostopic pub -1 /need_lists BehaviorModule/need_msg "{s1}need_name: \'{args.need}\', scene: \'\', person_name: \'\', IDtype: \'\', target_angle: 0.0, target_distance: 0.0, rob_emotion: \'\', rob_emotion_intensity: 0, person_emotion: \'\', weight: 0.0, speech: \'\', qt_order: 0, satisfy_value: 0, attitude: \'\', move_speed: 0.0, distance: 0.0, voice_speed: 0.0{s2}"'
    order_feed = f'rostopic pub -1 /BehaviorInstruction BehaviorModule/behavior_msg "{s1} header: {s1} seq: 0, stamp: {s1} secs: {args.time}, nsecs: 0 {s2} , frame_id: \'\'{s2}, name: \'{args.feedname}\', scene: \'\', type: \'\', total_phase: 0, current_phase: 0, occupancy: [0, 0, 0, 0, 0], target: \'\', target_angle: 0.0, target_distance: 0.0, speech: \'\', rob_emotion: \'\', rob_emotion_intensity: 0, attitude: \'\', move_speed: 0.0, distance: 0.0, voice_speed: 0.0{s2}"'

    if args.topic == "need":
        os.system(order_need)
    elif args.topic == "arm":
        os.system(order_arm)
    elif args.topic == "feed":
        os.system(order_feed)