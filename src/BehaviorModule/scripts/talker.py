#!/usr/bin/env python

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
    msg.current_phase = int(info[2])
    msg.total_phase = int(info[3])
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
