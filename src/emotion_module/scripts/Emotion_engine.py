#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import os
import numpy as np
import math
from bisect import bisect_left
import pandas as pd
import csv
import rospy
import matplotlib.pyplot as plt
plt.switch_backend('agg') 

#获取pkg目录
root=os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
fig = plt.figure(figsize=(9.6, 8.4)) # 图像像素大小为960*840

##### 实时参数初始化
emtionNumber = 8
delta_epre = np.zeros(emtionNumber)
delta_e = np.zeros(emtionNumber)      # 情绪增量向量 [ delta_emotion0 , delta_emotion1,... ]
current_e = np.zeros(emtionNumber) # 当前情感强度向量 [ float_emotion0 , emotion1,...]
current_m = 0.01                                              # 当前心境值,初始为0.01
delta_m = 0                                                      # 心境值增量
stimulus_t=30                                                   # 刺激持续时间,现默认所有刺激均持续15s
t_flag = 0                                                           # 判断是否处于刺激持续循环内
e_count=0                                                        # 记录情绪更新周期的参数
beta = 0.5

##### 情绪参数
tmp_lst = []
### 引用原始参数（新参数表是变化参数）
with open(os.path.join(root, 'scripts/csv/emotion_param.csv'), 'r') as f:
        reader = csv.reader(f)
        for row in reader:
                tmp_lst.append(row)
df = pd.DataFrame(tmp_lst[1:], columns=tmp_lst[0]) 
df.set_index(["param"], inplace=True)
df_list=df.values.tolist()
increaseKP=df_list[0]                              # 情感更新时持续时间延长系数（大于0小于1）
eci=[int(float(i))for i in df_list[1] ]       # 情绪类别
kmi=[float(i)for i in df_list[2] ]              # 心境对情绪的作用系数
a=df_list[3]                                                   # 幅度值
b=df_list[4]                                                   # 衰减系数
h0=0.01
### 设置人格
personality='extrovert'
## 设置内向introvert,外向与default的人格阈值
if personality == 'introvert':
    h1=[float(i)for i in df_list[5] ]                 # unaware情绪阈值为[h0,h1]
    h2=[float(i)for i in df_list[6] ]                 # mild情绪阈值为(h1,h2]
    h3=[float(i)for i in df_list[7] ]                 # moderate情绪阈值为(h2,h3]
    h4=[float(i)for i in df_list[8] ]                 # intense情绪阈值为(h3,h4]
elif personality == 'extrovert':
    h1=[float(i)for i in df_list[12] ]               
    h2=[float(i)for i in df_list[13] ]                 
    h3=[float(i)for i in df_list[14] ]                 
    h4=[float(i)for i in df_list[15] ]
else:
    h1=[float(i)for i in df_list[16] ]               
    h2=[float(i)for i in df_list[17] ]                 
    h3=[float(i)for i in df_list[18] ]                 
    h4=[float(i)for i in df_list[19] ]


beta=[float(i)for i in df_list[10] ]             # 人格变化参数
beta_star=[float(i)for i in df_list[11] ] # 人格变化参数*

##### 心境参数
d_m = 0.0077                                                # 衰减系数
b_m = 0                                                           # 情绪偏置,设置性格
kei = 0.2                                                           # 情绪对心境的作用系数,先默认在同向情绪下相同
updatecount = 5                                        # 情绪更新周期数,默认为10


class Emtion_engine:
    def __init__(self,current_e,current_m,delta_e):
        self.current_e = current_e
        self.current_m = current_m
        self.delta_e = delta_e


    def judge_mode(self):
        '''
        *刺激模式判断（对应四种情绪变化方式）
        '''
        mode=np.zeros(emtionNumber)
        for i in range(len(self.current_e)):
            flag = eci[i]*kmi[i]*self.current_m
            new_e = self.delta_e[i] + flag
            if new_e > 0:
                if new_e > self.current_e[i]:
                    mode[i] = 0 # 当前情绪值为前一时刻心境和当前刺激的综合作用
                else:
                    mode[i] = 1 # 仅延长了情绪体验时间
                # mode[i] = 1
            else:
                if (self.delta_e[i]+self.current_e[i])>abs(flag):
                    mode[i] = 2 # 情绪值为情绪与刺激间影响力更强的
                else:
                    mode[i] = 3 # 刺激不引起变化,保持自然衰减
        return mode


    
    def empathize_e(self,mode,current_t,start_t):
        '''
        *刺激下的移情规律计算
        :param mode: 该刺激将引发的情绪变化方式
        :param start_t: 刺激开始的时间,默认为唯一性队列队尾刺激的时间
        :param current_t: 当前时间,默认为当前计算的刺激消息对应的时间
        :output current_e: 直接更新全局变量（当前的情感强度）
        '''
        #print("case x.1")
        print(mode)
        delta_t=current_t-start_t
        if (delta_t)<1:
            delta_t=stimulus_t
        E=Emtion_engine(self.current_e,self.current_m,self.delta_e) # 创建移情计算实例
        ### 更新情绪强度值
        for i in range(len(self.current_e)):
            flag = eci[i]*kmi[i]*self.current_m
            if mode[i]==0:
                self.current_e[i] = self.delta_e[i]+flag
            if mode[i]==1:
                if self.current_e[i] > h0:
                    threshold = [h0,h1[i],h2[i],h3[i],h4[i]]
                    #h = bisect_left(threshold,self.current_e[i])
                    h = min(threshold, key=lambda x: abs(x - self.current_e[i]))
                    eai=(math.log(abs(self.current_e[i]))-math.log(threshold[int(h)]))/(float(increaseKP[i])*stimulus_t )* delta_t 
                    self.current_e[i] = self.current_e[i]*math.exp((-1)*eai) # 比自然衰减多了时间延长系数
            if mode[i]==2:
                # self.current_e[i] = [self.current_e[i],self.delta_e[i]][self.delta_e[i]>self.current_e[i]] 
                if self.current_e[i] > h0:
                    threshold = [h0,h1[i],h2[i],h3[i],h4[i]]
                    #h = bisect_left(threshold,self.current_e[i])
                    h = min(threshold, key=lambda x: abs(x - self.current_e[i]))
                    eai=(math.log(abs(self.current_e[i]))-math.log(threshold[int(h)]))/(float(increaseKP[i])*stimulus_t )* delta_t 
                    self.current_e[i] = self.current_e[i]*math.exp((-1)*eai) # 比自然衰减多了时间延长系数
            if mode[i]==3:
                self.current_e[i] = E.natural_attenuation_e(self.current_e[i],i,current_t,start_t) # 开启情感的自然衰减

            ### 限制情绪强度值的范围
            self.current_e[i] = max(min(0, max(self.current_e[i],h4[i])), min(self.current_e[i], h4[i])) 
        return self.current_e


    def natural_attenuation_e(self,e_val,i,current_t,start_t):
        '''
        *情感自然衰减（不在刺激影响时间范围内情绪值的变化）
        :param e_val: 传入的一个情感强度值（默认为某情感当前强度）
        :param start_t: 自然衰减开始的时间,可能是一个刺激结束的时间,也有可能是弱刺激开始的时间
        :param current_t: 当前时间,默认为当前计算的情感增量列表对应的时间
        :output e_val: 指数衰减后该情感的强度
        '''
        #print("case x.2")
        if (current_t-start_t)<1:
            e_val *=1
        else:
            if e_val > h0:
                threshold = [h0,h1[i],h2[i],h3[i],h4[i]] 
                #h = bisect_left(threshold,e_val) # 由情感值向下取第一个阈值
                h = min(threshold, key=lambda x: abs(x - self.current_e[i]))
                eai=(math.log(abs(e_val))-math.log(threshold[int(h)]))/stimulus_t * (current_t-start_t)
                e_val = e_val*math.exp((-1)*eai)
            else:
                e_val *=1
        return e_val


    def update_m(self):
        '''
        *心境变化与人格参数
        '''
        global e_count,delta_m
        for i in range(len(self.current_e)):
            l=np.nonzero(self.current_e)
            delta_m += eci[i]*kei*self.current_e[i]/(np.array(l).ndim*updatecount)
        e_count +=1 # e_count为记录情绪更新次数的参数
        if e_count == updatecount:
            self.current_m +=delta_m
            e_count = 0
            delta_m = 0
        else:
            self.current_m +=0
        return self.current_m
                       
    def natural_attenuation_m(self,current_tm,start_tm):
        self.current_m *= math.exp((-1)*d_m*(current_tm-start_tm))+ b_m
        return self.current_e

