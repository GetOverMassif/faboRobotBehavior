#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from re import I
import numpy as np
import math
from bisect import bisect_left
import pandas as pd
import csv
import rospy
import time
import threading
from social_msg.msg import attitude_msg
from social_msg.msg import perception_msg
from social_msg.msg import robot_emotion
from social_msg.msg import robot_status
import sys
import os

# "/home/zhjd/ws/src/social_system/emotion_module/scripts/"+'emotion_param.csv'
# /home/zhjd/ws/src/social_system/emotion_module/scripts/"+'emotion_param.csv

# csv_name = "/home/zhjd/ws/src/social_system/need_module/script/"+'test(school).csv'
# csv_name = "/home/zhjd/ws/src/social_system/need_module/script/"+'test(school).csv'
sys.stdout.write( '选择刺激输入源,【1】动机生成,【2】应急行为:  ' )
str = input();
current_folder_path = os.path.dirname(os.path.abspath(__file__))
print("Current folder path:", current_folder_path)
if str == 1 :
        print("Select motivation.csv")
        csv_name = "/home/zhjd/ws/src/social_system/need_module/script/"+'motivation.csv'
else:
    if str == 2 :
        print("Select emergency.csv")
        csv_name = "/home/zhjd/ws/src/social_system/need_module/script/"+'emergency.csv'
tmp_lst = []
with open(csv_name, 'r') as f:
        reader = csv.reader(f)
        i = 0
        for row in reader:
            if i == 0:
                i = 1
            else:    
                tmp_lst.append(row)

# df = pd.DataFrame(tmp_lst[1:], columns=tmp_lst[0]) 
# df.set_index(["person"], inplace=True)
# emotion_val=df.loc[1,1]
# t = emotion_val.split(',')
# print(tmp_lst[2][6])
# t =tmp_lst[2][6].split(' ')
# print(t)
# print(t[1])



def talker():
        rospy.init_node('need_third_test', anonymous=True)
        pub_perception = rospy.Publisher('perceptions', perception_msg, queue_size=10)
        pub_emotion = rospy.Publisher('robot_emotion', robot_emotion, queue_size=10)
        pub_body = rospy.Publisher('robot_status', robot_status, queue_size=10)
        # rate = rospy.Rate(5)
        rate = rospy.Rate(0.08) # 10s发一次
        # while not rospy.is_shutdown():
        i = 1
        for row in  tmp_lst:
                # if row[5] != '':
                        rate.sleep()
                        print(" ")
                        print(" ")
                        sys.stdout.write( '第 %s 次发布:\n' % row[0] )
                        i = i+1


                        emotion = robot_emotion()
                        temp = row[8].split(' ')
                        emotion.emotion1 = float(temp[0])
                        emotion.emotion2 = float(temp[1])
                        emotion.emotion3 = float(temp[2])
                        emotion.emotion4 = float(temp[3])
                        emotion.emotion5 = float(temp[4])
                        emotion.emotion6 = float(temp[5])
                        emotion.emotion7 = float(temp[6])
                        emotion.emotion8 = float(temp[7])
                        pub_emotion.publish(emotion)
                        # rospy.loginfo("emotion: %f,%f,%f,%f,%f,%f,%f,%f",emotion.emotion1,emotion.emotion2,emotion.emotion3,emotion.emotion4,emotion.emotion5,emotion.emotion6,emotion.emotion7,emotion.emotion8)
                        # print("情感状态:",end=' ')
                        # print(temp)
                        sys.stdout.write( "    情感状态:")
                        print(emotion.emotion1,emotion.emotion2,emotion.emotion3,emotion.emotion4,emotion.emotion5,emotion.emotion6,emotion.emotion7,emotion.emotion8)


                        body = robot_status()
                        temp = row[9].split(' ')
                        body.body1 = float(temp[0])
                        body.body2 = float(temp[1])
                        body.body3 = float(temp[2])
                        body.body4 = float(temp[3])
                        body.body5 = float(temp[4])
                        body.body6 = float(temp[5])
                        body.body7 = float(temp[6])
                        body.idleState = bool(temp[7])
                        pub_body.publish(body)
                        # rospy.loginfo("body: %f,%f,%f,%f,%f,%f,%f,%f",body.body1,body.body2,body.body3,body.body4,body.body5,body.body6,body.body7,body.body8)
                        sys.stdout.write( "    身体状态:")
                        print(body.body1,body.body2,body.body3,body.body4,body.body5,body.body6,body.body7,body.idleState)
    
                        time.sleep(0.5)
                        if row[5] != 0 :
                                per=perception_msg()
                                per.person_name = row[2]
                                per.IDtype = row[3]
                                per.intention = row[4]
                                per.p = float(row[5])
                                per.intention_2 = row[6]
                                per.p_2 = float(row[7])
                                pub_perception.publish(per)
                                # rospy.loginfo("perception: %s,%s,%s,%s",per.person_name,per.IDtype,per.intention,per.intention_2)
                                sys.stdout.write( "    感知信息:")
                                print(per.person_name,per.IDtype,per.intention,per.intention_2)
                        sys.stdout.write( "    正确的需求:")     
                        print(row[10])
                        
                        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass




