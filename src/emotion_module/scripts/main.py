#!/usr/bin/env python2
# -*- coding: UTF-8 -*-
import time
import copy
import rospy
import math
import sys
import signal
import threading
import Data_processing
from Data_processing import *
import message_filters
from Emotion_engine import *
from visualize_3d import *
import PIL.Image as Image
from social_msg.msg import attitude_set
from social_msg.msg import perception_msg
from social_msg.msg import attitude_query  
from social_msg.msg import idleState
class Watcher():
    '''
        *线程管理类,用于键盘结束多线程程序
    '''
    def __init__(self):
        self.child = os.fork()
        if self.child == 0:
            return
        else:
            self.watch()
 
    def watch(self):
        try:
            os.wait()
        except KeyboardInterrupt:
            self.kill()
        sys.exit()
 
    def kill(self):
        try:
            os.kill(self.child, signal.SIGKILL)
        except OSError:
            pass


class myThread(threading.Thread):
    def __init__(self,  name, content):
        threading.Thread.__init__(self)
        self.name = name
        self.content = content
    def run(self):
        print ("Starting ", self.name)
        if self.content == 'listener':
            data_get()
        elif self.content == 'publisher':
            while 1: 
                if rospy.is_shutdown():
                    break
                else:
                    time.sleep(1)
                    Data_processing.data_process()
        # else:
        #     visualize_3d()
 

def data_get():
        '''
        *启动模块,调用Data_processing.py中的函数,订阅三个刺激信息并作处理、发布robot_emotion_msg（进入循环）
        '''
        rospy.init_node('emotion_listener', anonymous=True,disable_signals=True)
        t1 = message_filters.Subscriber("attitude", attitude_set)
        t2 = message_filters.Subscriber("idleState",idleState)
        t3 = message_filters.Subscriber("perceptions", perception_msg)
        t4 = message_filters.Subscriber("idleState", idleState)
        # ts = message_filters.ApproximateTimeSynchronizer([t1, t3], 1, 1, allow_headerless=True)
        t1.registerCallback(Data_processing.callback_attitude)
        t2.registerCallback(Data_processing.callback_need)
        # ts.registerCallback(Data_processing.callback_a_p)
        t3.registerCallback(Data_processing.callback_perception)
        t4.registerCallback(Data_processing.callback_robot_status)
        rospy.spin() # spin() simply keeps python from exiting until this node is stopped


def Picture_Synthesis(mother_img,son_img,coordinate):
        """
        * 输入坐标,合成相应图片
        :param mother_img: 母图
        :param son_img: 子图
        :param coordinate: 子图在母图的坐标
        :output M_Img: 输出叠加后的图
        """
        ### 给图片指定色彩显示格式
        M_Img =mother_img.convert("RGBA")  # CMYK/RGBA 转换颜色格式（CMYK用于打印机的色彩,RGBA用于显示器的色彩）
        S_Img =son_img.convert("RGBA")
        r,g,b,a = S_Img.split() #分离出alpha通道作为mask,设置透明

        ### 获取图片的并调整
        M_Img_w, M_Img_h = M_Img.size  # 获取被放图片的大小（母图）
        S_Img_w, S_Img_h = S_Img.size  # 获取小图的大小（子图）
        factor = 1 # 子图缩小的倍数1代表不变,2就代表原来的一半
        S_Img_w=int(S_Img_w/factor)
        S_Img_h=int(S_Img_h/factor) 
        icon = S_Img.resize((S_Img_w, S_Img_h),Image.ANTIALIAS) #重新设置子图的尺寸
        ### 合并图片
        w = int((M_Img_w - S_Img_w) / 2)
        h = int((M_Img_h - S_Img_h) / 2)
        coordinate=(int(coordinate[0]-int(S_Img_w/2)),int(coordinate[1]-int(S_Img_h/2)))
        try:
            if coordinate==None or coordinate=="":
                coordinate=(w, h)
                ## 如果坐标指定错误,居中
                M_Img.paste(icon, coordinate, mask=a)
            else:
                ##  指定好坐标
                M_Img.paste(icon, coordinate, mask=a)
        except:
            print("坐标指定出错 ")
        # 保存图片
        return M_Img


def visualization():
        '''
        *可视化功能模块,计算各情绪小球坐标,绘图并在本地储存
        '''
        e_list =['{:.4f}'.format(i) for i in current_e]
        e_list=[float(i) for i in e_list] #当前情绪强度向量
       
        wheel_img=os.path.join(root, "image/Plutchik's_Wheel.png")
        img = Image.open(wheel_img)
        img_w, img_h = img.size
        midpoint=[img_w/2, img_h/2] #标记底图中点
        r = [60]*emtionNumber # 小球出发点半径（以midpoint为圆心,默认所有情绪相同）
        R = [265]*emtionNumber # 小球运动最大半径 （对应h4[i]）
        r_point = [(e_list[i] / h4[i]*(R[i]-r[i])+r[i]) for i in range(len(e_list))] 
        ## Joy
        cor_joy = (int(midpoint[0]),int(midpoint[1]-r_point[0]))
        final_img=Picture_Synthesis(img,son_img=Image.open(os.path.join(root, "image/Joy_ball.png")),
                  coordinate=(cor_joy))  
        ## Trust
        cor_trust = (int(midpoint[0]+r_point[1]/math.sqrt(2)),int(midpoint[1]-r_point[1]/math.sqrt(2)))
        final_img=Picture_Synthesis(final_img,son_img=Image.open(os.path.join(root, "image/Trust_ball.png")),
                  coordinate=(cor_trust))
        ## Surprise
        cor_surprise = (int(midpoint[0]-r_point[2]/math.sqrt(2)),int(midpoint[1]-r_point[2]/math.sqrt(2)))
        final_img=Picture_Synthesis(final_img,son_img=Image.open(os.path.join(root, "image/Surprise_ball.png")),
                  coordinate=(cor_surprise))
        ## Sadness
        cor_sadness = (int(midpoint[0]),int(midpoint[1]+r_point[3]))
        final_img=Picture_Synthesis(final_img,son_img=Image.open(os.path.join(root,"image/Sadness_ball.png")),
                  coordinate=(cor_sadness))
        ## Anger
        cor_anger = (int(midpoint[0]-r_point[4]),int(midpoint[1]))
        final_img=Picture_Synthesis(final_img,son_img=Image.open(os.path.join(root,"image/Anger_ball.png")),
                  coordinate=(cor_anger))
        ## Fear
        cor_fear = (int(midpoint[0]+r_point[5]),int(midpoint[1]))
        final_img=Picture_Synthesis(final_img,son_img=Image.open(os.path.join(root,"image/Fear_ball.png")),
                  coordinate=(cor_fear))
        ## Disgust
        cor_disgust = (int(midpoint[0]-r_point[6]/math.sqrt(2)),int(midpoint[1]+r_point[6]/math.sqrt(2)))
        final_img=Picture_Synthesis(final_img,son_img=Image.open(os.path.join(root,"image/Disgust_ball.png")),
                  coordinate=(cor_disgust))
        ## Boring
        cor_boring = (int(midpoint[0]+r_point[7]/math.sqrt(2)),int(midpoint[1]+r_point[7]/math.sqrt(2)))
        final_img=Picture_Synthesis(final_img,son_img=Image.open(os.path.join(root,"image/Boring_ball.png")),
                  coordinate=(cor_boring))

        final_img.save(os.path.join(root,"image/emotion_img.png"))

def visualize_3d():
    # global current_eq,m
    # # 判断两个消息列表中差集元素是否大于1
    # if len(set( m[-1]).symmetric_difference(set(current_e.insert(0,rospy.get_time()))))>0:
    #     update_visual(current_e,fig)
    #     m = copy.deepcopy(current_eq)
    global current_e
    while True:
        # print("Draw aready")
        c_e=copy.deepcopy(current_e)
    #     # update_visual(c_e,fig)
    
        

if __name__ == '__main__':
    Watcher()
    print("debug: new emotion module ")
    thread1=myThread("Listener-thread",'listener')
    thread2=myThread("Publisher-thread",'publisher')
    thread3=myThread("Visualize_3D",'visualize')
    thread1.start()
    thread2.start()
    # thread3.start()
