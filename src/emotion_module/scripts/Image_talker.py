#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import Image
import cv2
import numpy as np
import time
IMAGE_WIDTH=1280
IMAGE_HEIGHT=720

rospy.init_node('Image_talker', anonymous=True)
pub = rospy.Publisher('emotion_img', Image, queue_size=1)
rate = rospy.Rate(1) # 1hz

def publish_image(imgdata):
    while not rospy.is_shutdown():
        image_temp=Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = 'map'
        image_temp.height=IMAGE_HEIGHT
        image_temp.width=IMAGE_WIDTH
        image_temp.encoding='rgb8'
        image_temp.data=np.array(imgdata).tostring()
        #print(imgdata)
        #image_temp.is_bigendian=True
        image_temp.header=header
        image_temp.step=1241*3
        pub.publish(image_temp)
        rospy.loginfo("图片成功发布")
        rate.sleep()

if __name__ == '__main__':
    try:
        img=cv2.imread('/home/zzzane/robot_empathy/src/emotion_module/image/emotion_img')
        publish_image(img)
    except rospy.ROSInterruptException:
        pass




