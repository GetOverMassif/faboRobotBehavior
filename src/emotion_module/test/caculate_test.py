import math
import time
import numpy as np
def natural_attenuation_e(e_val,i,current_t,start_t):
        '''
        *情感自然衰减（不在刺激影响时间范围内情绪值的变化）
        :param e_val: 传入的一个情感强度值（默认为某情感当前强度）
        :param start_t: 自然衰减开始的时间,可能是一个刺激结束的时间,也有可能是弱刺激开始的时间
        :param current_t: 当前时间,默认为当前计算的情感增量列表对应的时间
        :output e_val: 指数衰减后该情感的强度
        '''
        global stimulus_t,h1,h2,h3,h4
        #print("case x.2")
        if (current_t-start_t)<1:
            e_val *=1
        else:
            if e_val > h1:
                threshold = [h1,h2,h3,h4] 
                #h = bisect_left(threshold,e_val) # 由情感值向下取第一个阈值
                h = min(threshold, key=lambda x: abs(x - current_e[i]))
                eai=(math.log(abs(e_val))-math.log(threshold[int(h)]))/stimulus_t * (current_t-start_t)
                e_val = e_val*math.exp((-1)*eai)
            else:
                e_val *=1
        return e_val


current_e=[0,0,0,0,0,0,0,0.6]
start_time=time.time()
stimulus_t =5
h1=0
h2=0.1
h3=0.4
h4=0.7
while np.any(current_e):
    for i in range(len(current_e)):
        current_t=time.time()
        current_e[i]=natural_attenuation_e(current_e[i],i,current_t,start_time)
    print(current_e)
    time.sleep(1)