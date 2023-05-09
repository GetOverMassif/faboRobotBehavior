#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import time
from social_msg.msg import robot_status

def robot_status_talker():
    rospy.init_node('robot_status_talker', anonymous=True)
    pub = rospy.Publisher('robot_status', robot_status, queue_size=10)
    time.sleep(2)
    
    status=robot_status()
    status.idleState = 1
    pub.publish(status)
    rospy.loginfo("当前机器人闲置状态: %s",status.idleState)
    rospy.spin()
    
    # num = 0
    # rate = rospy.Rate(1) # 1hz
    # while not rospy.is_shutdown():
    #     status=robot_status()
    #     if num<1 :
    #         status.idleState = 1
    #         num += 1
    #     else:
    #         status.idleState = 0
    #     pub.publish(status)
    #     rospy.loginfo("当前机器人闲置状态: %s",status.idleState)
    #     rate.sleep()


if __name__ == '__main__':
    try:
        robot_status_talker()
    except rospy.ROSInterruptException:
        pass
