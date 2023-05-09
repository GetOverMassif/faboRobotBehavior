#!/usr/bin/env python2
# -*- coding:utf-8 -*-

import csv
import sys
# import importlib
reload(sys) 
sys.setdefaultencoding('utf-8')
import copy
import time
import numpy as np
import pandas as pd
from sentiment import *
import rospy
import main
from Emotion_engine import *
from visualize_3d import *
from social_msg.msg import robot_emotion
from collections import deque
from social_msg.msg import attitude_query  

##### 全局参数初始化
local_attitude_set =[]
need_eval=[]
attitude_eval=[]
perception_eval=[]
delta_elist=[]                                         # 总情绪变化列表 [ float_情绪种类,float_情绪变化强度... ]
msg_list=[time.time(), 'None', 0, 'None', 'None', 'None', 'enthusiastic', 'None']      # msg列表,记录每时刻接收到刺激的内容
# msg_list[6] is attitude.attitude ;    msg_list[5] is perception_msg.person_emotion
unique_msg=deque(maxlen=10) # 消息的唯一性队列,存储被判定为不同刺激的消息内容
msg_dt=10000                                             # 两次内容相同消息,但作为不同刺激输入的最小时间阈值
e_rec=[]                                                   # 记录情感变化值数组

##### 配置百度文本情感趋势分析接口
# APP_ID = '25295465'
# API_KEY = 'WxY0AiFTs0HgWZaSsWGB6lM4'
# SECRET_KEY = '9oGOc6MmGGkFcYp6aGaRRy1fT87TYQ1t'
# client = AipNlp(APP_ID, API_KEY, SECRET_KEY)   

# 身体状态对无聊情绪的影响
# global idleState_last, idleState_flag, time_init, time_cur, idleState_to_boring
time_init   = 0
idleState_flag = 0

def callback_robot_status( idleState ):  
       '''
       * 身体状态信息处理,用于“无聊情绪”
       :param robot_status_msg: 订阅自我满足需求信息
       :output : 身体状态 引起的情绪变化列表？？？[ float_情绪种类,float_情绪变化强度... ]
       '''
       print("接收robot_status_msg ")
       rospy.loginfo( "I heard %s ", idleState.idleState )
       global idleState_last, idleState_flag, time_init, time_cur, idleState_to_boring  

       # 现在的: 
       if ( idleState.idleState == 1 ):
              print("接收到robot 处于闲置状态")
              time_init = time.time()
              idleState_flag = 1
       else:
              idleState_flag = 0
       # idleState_last = robot_status_msg.idleState



##### 发布话题 
pub_query = rospy.Publisher('attitude_query', attitude_query, queue_size=1)
pub = rospy.Publisher('robot_emotion', robot_emotion, queue_size=10)
def publish():
       '''
       robot_emotion消息发布
       '''
       eval=robot_emotion()
       global current_e
       eval.emotion1 = current_e[0]
       eval.emotion2 = current_e[1]
       eval.emotion3 = current_e[2]
       eval.emotion4 = current_e[3]
       eval.emotion5 = current_e[4]
       eval.emotion6 = current_e[5]
       eval.emotion7 = current_e[6]
       eval.emotion8 = current_e[7]
       pub.publish(eval)
       rospy.loginfo("机器人情感: %f,%f,%f,%f,%f,%f,%f,%f\n", eval.emotion1,eval.emotion2,eval.emotion3,eval.emotion4,eval.emotion5,eval.emotion6,eval.emotion7,eval.emotion8 )


def caculate_edelta(csv_name,index_name,index_val,column_val):
       '''
       * 通过刺激消息计算情绪变化强度值
       :param csv_name: 对应消息内容计算规则csv文档路径
       :param index_name: 作为dataframe的index的列名
       :param index_val: 传入的消息内容中对应行索引值大小
       :param column_val: 传入的消息内容中对应列索引值大小
       :output t: 依据消息和csv文档索引得到的情绪变化强度值
       '''
       tmp_lst = []
       with open(csv_name, 'r') as f:
              reader = csv.reader(f)
              for row in reader:
                     tmp_lst.append(row)
       df = pd.DataFrame(tmp_lst[1:], columns=tmp_lst[0]) 
       df.set_index([index_name], inplace=True)
       emotion_val=df.loc[index_val,column_val]
       t = emotion_val.split(',')
       return t


def unique_judge(msg):
       '''
       * 判断msg_list是否在唯一性队列中
       * 更新唯一性队列
       :param msg: 待判断的消息内容
       :output flag: 该msg是否构成刺激(0 & 1)
       '''
       global unique_msg,msg_dt
       msg_init=[rospy.get_time(), 'None', 0, 'None', 'None', 'None', 'enthusiastic', 'None']
       if len(unique_msg)==0:
              unique_msg.append(msg)
              flag=1
       # 判断msg_list是否为初始状态,非初始状态则进行唯一性判断
       elif len(set(msg_init).symmetric_difference(set(msg)))<3:
              flag=0
       else:
              #print("unique_msg:",unique_msg)
              flag=0;
              l=copy.deepcopy(unique_msg)
              # 判断两个消息列表中差集元素是否大于1
              if len(set(l[-1]).symmetric_difference(set(msg)))>2:
                     unique_msg.append(msg)
                     flag=1
              elif len(set(l[-1]).symmetric_difference(set(msg)))==2:
                     if abs(msg[0]-l[-1][0])>=msg_dt:
                            unique_msg.append(msg)
                            flag=1
              else:
                     for item in unique_msg:
                            if item[1]!=msg[1] and item[3]==msg[3]:
                                   if msg[0]-item[0]<60:
                                          flag=0
                                   else:
                                          flag=1
       return flag


def delta_echange(need_eval,attitude_eval,perception_eval):
       '''
       *情绪增量初始化
       :param need_eval、attitude_eval、perception_eval: 三种刺激带来的情绪增量
       :output deta_e: 标准化的情绪增量向量 [ delta_emotion0 , delta_emotion1,... ]
       '''
       delta_elist=need_eval+attitude_eval+perception_eval
       for i in range(len(delta_elist[::2])):
              delta_e[int(delta_elist[::2][i])]+=delta_elist[1::2][i]


def caculate_e(delta_e,delta_epre):
       '''
       *开启情感计算,共分为5种情况调用
       :param delta_e : 直接更新全局变量（情绪增量向量）
       :param delta_epre : 上一时刻情绪增量向量
       '''
       global current_e,current_m,t_flag,msg_list,stimulus_t
       E=Emtion_engine(current_e,current_m,delta_e) # 创建移情计算实例

       ### 判断是否在刺激持续时间内
       current_t= msg_list[0]
       if current_t<=(unique_msg[-1][0]+stimulus_t):   
              t_flag=1   # t_flag=1标记在刺激持续时间内
       else:
              t_flag=0
       #print("t_flag: ",t_flag)
       ### 情绪增量全零情况
       if not(np.any(delta_e)): 
              if t_flag == 0:
                     print(t_flag)
                     print("case1")
                     #print("delta_time:", current_t-unique_msg[-1][0]-stimulus_t)
                     for i in range(len(current_e)):
                            current_e[i]=E.natural_attenuation_e(current_e[i],i,current_t,unique_msg[-1][0]+stimulus_t)
                     current_m=E.update_m()   
              else:
                     delta_e = copy.deepcopy(delta_epre)
                     ## 情绪强度向量全零情况
                     if not(np.any(current_e)): 
                            print("case2")
                            current_e+=delta_e #直接加和情绪增量
                            for i in range(len(current_e)):
                                   current_e[i] = max(min(0, max(current_e[i],h4[i])), min(current_e[i], h4[i])) 
                            current_m=E.update_m() 
                     ## 情绪强度向量非零情况
                     else:
                            print("case3")
                            print('current_t:',current_t)
                            print('start_t:',unique_msg[-1][0])
                            mode=E.judge_mode()
                            current_e=E.empathize_e(mode,current_t,unique_msg[-1][0])
                            current_m=E.update_m()  
       ### 情绪增量非零情况
       else:
              ## 情绪强度向量全零情况
              if not(np.any(current_e)): 
                     print("case4")
                     current_e+=delta_e #直接加和情绪增量,形成情绪突跳,对心境无影响
                     for i in range(len(current_e)):
                            current_e[i] = max(min(0, max(current_e[i],h4[i])), min(current_e[i], h4[i]))
                     current_m=E.update_m() 
              ## 情绪强度向量非零情况
              else:
                     print("case5")
                     mode=E.judge_mode()
                     current_e=E.empathize_e(mode,current_t,unique_msg[-1][0])
                     current_m=E.update_m()      
                                          
                           
                     
def callback_need(idleState):
       '''
       * 自我满足信息处理
       :param need_satisfy_msg: 订阅自我满足需求信息
       :output need_eval: 自我需求满足引起的情绪变化列表[ float_情绪种类,float_情绪变化强度... ]
       '''
       #rospy.loginfo( "I heard %s %d", need_satisfy_msg.need_name,need_satisfy_msg.satisfy_value)

       ### 更新msg列表
       # msg_list.append(need_satisfy_msg.need_name) 
       # msg_list.append(need_satisfy_msg.satisfy_value) 
       global need_eval,msg_list 
       msg_list[1]=idleState.hehavior_name
       msg_list[2]=idleState.satisfy_value

       #### 自我满足信息处理,通过查找dataframe实现值映射
       csv_name  = os.path.join(root,'scripts/csv/need_satisfy.csv')
       index_name = "satisfy_value"
       index_val = str(idleState.satisfy_value)
       column_val = idleState.hehavior_name
       ### 由  自我状态满足  带来的情绪变化状况 [ float_情绪种类,float_情绪变化强度 ]
       need_eval= map(float,caculate_edelta(csv_name,index_name,index_val,column_val)) 


# def callback_a_p(attitude_msg,perception_msg):  
#        '''
#        *更新msg列表和情绪增量列表
#        *处理用户情绪迁移 + 用户语音输入信息（这两种刺激会作时间对齐处理）
#        *调用移情计算
#        *开启可视化、机器人情感topic发布
#        '''
#        #rospy.loginfo( "I heard %s %s %s %s %s",perception_msg.person_name,perception_msg.speech,\
#               #perception_msg.person_emotion,attitude_msg.person_name,attitude_msg.attitude)

#        #### 更新msg列表
#        global msg_list
#        msg_list.extend([perception_msg.person_name,attitude_msg.person_name,\
#                                           perception_msg.person_emotion,attitude_msg.attitude,perception_msg.speech])
#        msg_list.insert(0,perception_msg.time)
def callback_attitude(attitude_set):      
       # rospy.loginfo( "I heard %s %s ",attitude_msg.person_name,attitude_msg.attitude)
       
       # version 1: single and attitude
       # global msg_list
       # msg_list[4]=attitude_msg.person_name
       # msg_list[6]=attitude_msg.attitude
       # # msg_list.extend([attitude_msg.person_name,attitude_msg.attitude])

       # version 2: attitude set
       global local_attitude_set
       local_attitude_set = []
       for attitude in attitude_set.attitudes:
              local_attitude_set.append(attitude)


def callback_perception(perception_msg): 
       global msg_list

       # version1:  query attitude
       query = attitude_query()
       query.person_name = perception_msg.person_name
       query.IDtype  = perception_msg.IDtype 
       query.motivation = "Greet"
       pub_query.publish(query)
       rospy.loginfo(" 发送查询Greet的社交态度\n")

       # version2:  查询本地列表。默认是热情
       msg_list[4]='none'
       msg_list[6]='enthusiastic'
       for attitude in local_attitude_set:
              if attitude.person_name == perception_msg.person_name    and attitude.IDtype  == perception_msg.IDtype     and attitude.motivation  == 'Greet':
                     msg_list[4]=attitude.person_name
                     msg_list[6]=attitude.attitude
                     # msg_list.extend([attitude_msg.person_name,attitude_msg.attitude])           
       # msg_list.insert(2,perception_msg.person_name)
       # msg_list.insert(4,perception_msg.person_emotion)
       # msg_list.append(perception_msg.speech)
       # msg_list.insert(0,perception_msg.time)
       msg_list[3]=perception_msg.person_name
       msg_list[5]=perception_msg.person_emotion
       # reload(sys) 
       # sys.setdefaultencoding('utf-8')
       # msg_list[7]=perception_msg.speech.encode().decode("raw_unicode_escape") 
       msg_list[7]=perception_msg.speech
       # msg_list[7].encode('raw_unicode_escape').decode()
       msg_list[0]=perception_msg.time


def data_process():
       global msg_list,current_e,current_m,delta_e,delta_epre
       # if  (not msg_list):
       #        msg_list=[rospy.get_time(), 'None', 0, 'None', 'None', 'None', 'enthusiastic', 'None']
       # elif len(msg_list)==2:
       #        msg_list.insert(0,rospy.get_time())
       #        msg_list.extend(['None', 'None', 'None', 'enthusiastic', 'None'])
       # elif len(msg_list)!=8:
       #        msg_list.insert(1,0)
       #        msg_list.insert(1,'None')
       print("Msg list: ",msg_list) 
       # print('u'+msg_list[7])       
       flag=unique_judge(msg_list)
       print("Whether_work: ",flag)
       
       #### 更新情绪增量列表
       if flag :
              #print('Internal_eval: ',need_eval)
              ### 社交态度信息处理
              # if msg_list[3] == msg_list[4]:
              #        csv_name  = os.path.join(root, 'scripts/csv/attitude.csv')
              #        index_name = "person_emotion"
              #        index_val = msg_list[5]
              #        column_val = msg_list[6]
              #        ## 由  社交态度  带来的情绪变化状况 [ float_情绪种类,float_情绪变化强度 ]
              #        global perception_eval,need_eval,attitude_eval
              #        attitude_eval= map(float,caculate_edelta(csv_name,index_name,index_val,column_val)) 
              #        #print ('PeopleEmotionTransfer_eval: ',attitude_eval)                
              # else:
              #        print("The social attitude object is different from the conversation object ! ")
              csv_name  = os.path.join(root, 'scripts/csv/attitude.csv')
              index_name = "person_emotion"
              
              index_val = msg_list[5]     # msg_list[5] is perception_msg.person_emotion
              column_val = msg_list[6]    # msg_list[6] is attitude.attitude 
              
              ## 由  社交态度  带来的情绪变化状况 [ float_情绪种类,float_情绪变化强度 ]
              global perception_eval,need_eval,attitude_eval
              attitude_eval= map(float,caculate_edelta(csv_name,index_name,index_val,column_val)) 
              #print ('PeopleEmotionTransfer_eval: ',attitude_eval)              



              ### 他人评价信息处理
              if 'u'+msg_list[7]=='u'+'None':
                     eclass= 'none'
              else:
                     text = msg_list[7]
                     # charset="UTF-8"
                     # try:
                     #        eclass=client.sentimentClassify(text)[u'items'][0][u'sentiment'] #文本对应情感类别标签
                     # except:
                     #        print("API failed !")
                     #        eclass='none'
                     try:
                            senti = Sentiment()
                            result = senti.sentiment_count(text)
                            if result['pos']!=0:
                                   eclass=2
                            elif result['neg']!=0:
                                   eclass=0
                            else:
                                   eclass=1
                     except:
                            print("Emotion recognition failed !")
                            eclass='none'
                     # senti=Sentiment()
                     # result = senti.sentiment_count("你真棒")
                     # print("result:",result)
                     
              csv_name1  =  os.path.join(root,'scripts/csv/perception.csv')
              index_name1 = "speech_emotion"
              index_val1 = str(eclass)
              column_val1 = 'empathy'
              ## 由  他人评价 带来的情绪变化状况 [ float_情绪种类,float_情绪变化强度 ]
              perception_eval= map(float,caculate_edelta(csv_name1,index_name1,index_val1,column_val1)) 
              #print('External_eval: ',perception_eval)

              ### 情绪增量标准化
              delta_epre = copy.deepcopy(delta_e) # 记录上一时刻的delta_e
              delta_echange(need_eval,attitude_eval,perception_eval)
       else:
              delta_e = np.zeros(emtionNumber) # 当信息不构成刺激时,情绪增量向量为0向量
       print("List of emotion change: ",delta_e)

       
       
       # zhjd 闲置状态对无聊情绪的影响
       global  idleState_flag, time_init, time_cur,t_flag,stimulus_t
       if idleState_flag :
              #print("运行robot_status_msg to boring ")
              time_cur = time.time()
              print("机器人~~~~~~闲置了%f秒"%(time_cur - time_init)) 
              if ( (time_cur - time_init) > 30 ):
                     delta_e[7] = 0.6   #current_e  delta_e
                     unique_msg.append([rospy.get_time()-stimulus_t, 'None', 0, 'None', 'None', 'None', 'enthusiastic', 'None'])
                     idleState_flag  = 0
                     print("无聊情绪置为 0.6")
      
       #### 调用移情计算
       caculate_e(delta_e,delta_epre)
       msg_list=[rospy.get_time(), 'None', 0, 'None', 'None', 'None', 'enthusiastic', 'None']

       #### 情感可视化
       main.visualization()
       #main.visualize_3d()
       # global current_eq
       # current_eq.append(current_e.insert(0,rospy.get_time()))

       print("current_mood:",current_m)       
       #### 机器人情感topic发布
       publish()
       
