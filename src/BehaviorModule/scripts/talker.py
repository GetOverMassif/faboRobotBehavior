#!/usr/bin/env python
# import argparse
# import os

# if __name__=="__main__":
#     parser = argparse.ArgumentParser(description="Hello")
#     parser.add_argument("--topic", type=str, default="need")
#     parser.add_argument("--action", type=str, default="wave")
#     parser.add_argument("--need", type=str, default="Greet")
#     parser.add_argument("--feedname", type=str, default="Greet")
#     parser.add_argument("--time", type=int, default=0)

#     args = parser.parse_args()

#     s1 = "{"
#     s2 = "}"

#     order_arm = f'rostopic pub -1 /arm_action arm_control/Arms "{s1}call: false, weight: 0, action: \'{args.action}\', rate: 0, angle: 0, startTime: 0, endTime: 0, IsYellow: false{s2}"'
#     order_need = f'rostopic pub -1 /need_lists BehaviorModule/need_msg "{s1}need_name: \'{args.need}\', scene: \'\', person_name: \'\', IDtype: \'\', target_angle: 0.0, target_distance: 0.0, rob_emotion: \'\', rob_emotion_intensity: 0, person_emotion: \'\', weight: 0.0, speech: \'\', qt_order: 0, satisfy_value: 0, attitude: \'\', move_speed: 0.0, distance: 0.0, voice_speed: 0.0{s2}"'
#     order_feed = f'rostopic pub -1 /BehaviorFeedback BehaviorModule/behavior_feedback_msg "header: {s1} seq: 0, stamp: {s1} secs: {args.time}, nsecs: 0 {s2} , frame_id: \'\'{s2}, hehavior_name: \'Greet\', total_phase: 2, current_phase: 2"'

#     if args.topic == "need":
#         os.system(order_need)
#     elif args.topic == "arm":
#         os.system(order_arm)
#     elif args.topic == "feed":
#         os.system(order_feed)
import rospy
from BehaviorModule.msg import need_msg, behavior_msg, behavior_feedback_msg, idleState

def colored(str):
    return f"\033[0;34;40m{str}\033[0m"

def editNeedMsg():
    msg = need_msg()
    message = input(" - need_name,person_name : \n   ")
    info = message.split(',')
    msg.need_name = info[0]
    msg.person_name = info[1]
    return msg

def editBehaviorOrder():
    msg = behavior_msg()
    message = input(" - name,target,current_phase,total_phase : \n   ")
    info = message.split(',')
    msg.name = info[0]
    msg.target = info[1]
    msg.current_phase = int(info[2])
    msg.total_phase = int(info[3])
    return msg

def editBehaviorFeedback():
    msg = behavior_feedback_msg()
    message = input(" - hehavior_name,stamp(sec),current_phase,total_phase : \n   ")
    info = message.split(',')
    msg.header.stamp.secs = int(info[1])
    msg.hehavior_name = info[0]
    msg.current_phase = int(info[3])
    msg.total_phase = int(info[2])
    return msg

def editIdleStateMsg():
    msg = idleState()
    message = input(" - idleState(0/1),hehavior_name : \n   ")
    info = message.split(',')
    msg.idleState = info[0]
    msg.hehavior_name = info[1]
    return msg

class Talker:
    def __init__(self) -> None:
        self.pub_need = rospy.Publisher('/need_lists', need_msg, queue_size=10)
        self.pub_beh = rospy.Publisher('/BehaviorInstruction', behavior_msg, queue_size=10)
        self.pub_feed = rospy.Publisher('/BehaviorFeedback', behavior_feedback_msg, queue_size=10)
        self.pub_state = rospy.Publisher('/idleState', idleState, queue_size=10)
        self.rate = rospy.Rate(10)
        print("")
        while not rospy.is_shutdown():
            print(colored("1-Need, 2-BehaviorOrder, 3-BehaviorFeedback, 4-IdleState, 0-Exit"))
            topic = int(input(" - Choose topic (1/2/3/4): "))
            if topic == 1:
                msg = editNeedMsg()
                self.pub_need.publish(msg)
            elif topic == 2:
                msg = behavior_msg()
            elif topic == 3:
                msg = editBehaviorFeedback()
                self.pub_feed.publish(msg)
            elif topic == 4:
                msg = idleState()
            elif topic == 0:
                break
            else:
                print("Error: please input 1/2/3/4")
            print("")
            self.rate.sleep()

if __name__=="__main__":
    rospy.init_node('talker')
    try:
        Talker()
    except rospy.ROSInternalException:
        pass
