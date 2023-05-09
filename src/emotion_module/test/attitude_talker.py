#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import time
from social_msg.msg import attitude_msg
from social_msg.msg import perception_msg

def attitude_talker():
    rospy.init_node('attitude_talker', anonymous=True)
    pub = rospy.Publisher('attitude', attitude_msg, queue_size=10)
    
    atti=attitude_msg()
    atti.person_name ="Teacher_Li"
    atti.attitude = "enthusiastic"
    pub.publish(atti)
    rospy.loginfo("subscribe1: %s,%s",atti.person_name,atti.attitude)
    pub2 = rospy.Publisher('perceptions', perception_msg, queue_size=10)
    
    perc = perception_msg()
    perc.time = rospy.get_time()
    perc.person_name ="Teacher_Li"
    perc.speech = "你真棒!"
    perc.person_emotion = "Happy"
    pub2.publish(perc)
    rospy.loginfo("发布的消息2: %f,%s,%s,%s",perc.time,perc.person_name,perc.speech,perc.person_emotion)
    rospy.spin()
    
    # rate = rospy.Rate(1) # 1hz
    # while not rospy.is_shutdown():
    #     atti=attitude_msg()
    #     atti.person_name ="Teacher_Li"
    #     atti.attitude = "enthusiastic"
    #     pub.publish(atti)
    #     rospy.loginfo("subscribe1: %s,%s",atti.person_name,atti.attitude)
    #     rate.sleep()

if __name__ == '__main__':
    try:
        attitude_talker()
    except rospy.ROSInterruptException:
        pass










