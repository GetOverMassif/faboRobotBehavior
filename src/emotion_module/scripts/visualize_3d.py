# !/usr/bin/env python
# -*- coding: UTF-8 -*-
import os
import csv
import pandas as pd
import math
from bisect import bisect_left
import matplotlib.pyplot as plt
# plt.switch_backend('agg') 
import numpy as np
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

### 获取pkg目录
root=os.path.dirname(os.path.abspath(os.path.dirname(__file__)))

def getLinearEquation(p1x, p1y, p2x, p2y):
        '''
        *根据已知两点坐标,求过这两点的直线解析方程:  a*x+b*y+c = 0  (a >= 0)
        '''
        sign = 1
        a = p2y - p1y
        if a < 0:
                sign = -1
                a = sign * a
        b = sign * (p1x - p2x)
        c = sign * (p1y * p2x - p1x * p2y)
        return [a, b, c]


def update_visual(current_e,fig,ii):
        #fig.clf()
        plt.clf()
        ### 读取x,y,z
        tmp_lst = []
        with open(os.path.join(root, 'scripts/csv/personality.csv'), 'r') as f:
                reader = csv.reader(f)
                for col in reader:
                        tmp_lst.append(col)
        df = pd.DataFrame(tmp_lst[1:], columns=tmp_lst[0]) 
        df.set_index(["param"], inplace=True)
        df_list=df.values.tolist()
        x_introvert=df_list[0]
        x=list(map(float,x_introvert))
        y_introvert=df_list[1]
        y=list(map(float,y_introvert))
        z_introvert=df_list[3]
        z=list(map(float,z_introvert))
        zz =  [z[i]-z[i]-0.3 for i in range(0,len(z))]
        ### 三维图画布
        sub=fig.add_subplot(111,projection='3d')
        ### 轴标签与取值范围
        sub.set_xlabel(r'$h_{ijx}$')
        sub.set_ylabel(r'$h_{ijy}$')
        sub.set_zlabel(r'$S_{eij}$')
        plt.xlim(-1, 1)
        plt.ylim(-1, 1)
        sub.set_zlim(-0.3,1)
        ### 绘制线
        p_00,p_11, p_12,p_13,p_14,p_21, p_22,p_23,p_24,p_31, p_32,p_33,p_34,p_41, p_42,p_43,p_44,\
        p_51,p_52,p_53,p_54,p_61, p_62,p_63,p_64,p_71, p_72,p_73,p_74,p_81, p_82,p_83,p_84 = zip(x, y, z) # 立体点
        pd_00,pd_11, pd_12,pd_13,pd_14,pd_21, pd_22,pd_23,pd_24,pd_31, pd_32,pd_33,pd_34,pd_41, pd_42,pd_43,pd_44,\
        pd_51,pd_52,pd_53,pd_54,pd_61, pd_62,pd_63,pd_64,pd_71, pd_72,pd_73,pd_74,pd_81, pd_82,pd_83,pd_84 = zip(x, y,zz) # 投影点
        ## 投影连线
        line_mild = zip(pd_11,pd_21,pd_61,pd_81,pd_41,pd_71,pd_51, pd_31,pd_11)
        line_moderate = zip(pd_12,pd_22,pd_62,pd_82,pd_42,pd_72,pd_52, pd_32,pd_12)
        line_intense = zip(pd_13,pd_23,pd_63,pd_83,pd_43,pd_73,pd_53, pd_33,pd_13)
        line_max = zip(pd_14,pd_24,pd_64,pd_84,pd_44,pd_74,pd_54, pd_34,pd_14)
        sub.plot( [list(i) for i in line_mild][0],[list(i) for i in line_mild][1],[list(i) for i in line_mild][2],'--',label='Mild', color='#999999', linewidth=1)
        sub.plot( [list(i) for i in line_moderate][0],[list(i) for i in line_moderate][1],[list(i) for i in line_moderate][2],'--',label='Moderate', color='#666666', linewidth=1)
        sub.plot( [list(i) for i in line_intense][0],[list(i) for i in line_intense][1],[list(i) for i in line_intense][2],'--',label='Intense', color='#333333', linewidth=1)
        sub.plot( [list(i) for i in line_max][0],[list(i) for i in line_max][1],[list(i) for i in line_max][2],'--',label='Max',  color='#000000', linewidth=1)
        #sub.plot( '--',label='Mild', color='#999999', linewidth=1,*line_mild)
        # sub.plot3D( '--', label='Moderate', color='#666666', linewidth=1,*line_moderate)
        # sub.plot3D('--', label='Intense', color='#333333', linewidth=1,*line_intense)
        # sub.plot3D('--',label='Max',  color='#000000', linewidth=1,*line_max )
        # sub.plot3D( *line_mild,'--',label='Mild', color='#999999', linewidth=1)
        # sub.plot3D(*line_moderate, '--', label='Moderate', color='#666666', linewidth=1)
        # sub.plot3D(*line_intense,'--', label='Intense', color='#333333', linewidth=1)
        # sub.plot3D(*line_max,'--',label='Max',  color='#000000', linewidth=1 )
        ## Joy
        lines_0 = zip(p_00,p_11, p_12,p_13,p_14)
        sub.plot3D(label='Joy', color='#fcb001', linewidth=3,zdir='z',    
                marker='o',    # 标记点符号
                mfc='#fcb001',    # marker facecolor
                mec='#fcb001',    # marker edgecolor
                ms=10,*lines_0)   # size
        lines_d0 = zip(pd_00,pd_11, pd_12,pd_13,pd_14)     
        sub.plot( color='#fcb001', linewidth=2,marker='o',    
                mfc='#fcb001',   mec='#fcb001',   ms=4,*lines_d0)  # 投影
        ## Trust
        lines_1 = zip(p_00,p_21, p_22,p_23,p_24)
        sub.plot3D(label='Trust', color='#9ce143', linewidth=3,zdir='z',    marker='o',    
                mfc='#9ce143',   mec='#9ce143',   ms=10,*lines_1) # 立体
        lines_d1 = zip(pd_00,pd_21, pd_22,pd_23,pd_24)     
        sub.plot(color='#9ce143', linewidth=2,marker='o',    
                mfc='#9ce143',   mec='#9ce143',   ms=4,*lines_d1)  # 投影
        ## Antipation
        lines_2 = zip(p_00,p_31, p_32,p_33,p_34)
        sub.plot3D(label='Antipation', color='#ff5b00', linewidth=3,zdir='z',    marker='o',    
                mfc='#ff5b00',   mec='#ff5b00',   ms=10,*lines_2) 
        lines_d2 = zip(pd_00,pd_31, pd_32,pd_33,pd_34)
        sub.plot3D(color='#ff5b00', linewidth=2,   marker='o',    
                mfc='#ff5b00',   mec='#ff5b00',   ms=4,*lines_d2)         
        ## Sadness
        lines_3 = zip(p_00,p_41, p_42,p_43,p_44)
        sub.plot3D(label='Sadness', color='#0e87cc', linewidth=3,zdir='z',    marker='o',    
                mfc='#0e87cc',   mec='#0e87cc',   ms=10,*lines_3) 
        lines_d3 = zip(pd_00,pd_41, pd_42,pd_43,pd_44)
        sub.plot3D(color='#0e87cc', linewidth=2,    marker='o',    
                mfc='#0e87cc',   mec='#0e87cc',   ms=4,*lines_d3 ) 
        ## Anger
        lines_4 = zip(p_00,p_51,p_52,p_53,p_54)
        sub.plot3D(label='Anger', color='#ff0789', linewidth=3,zdir='z',    marker='o',    
                mfc='#ff0789',   mec='#ff0789',   ms=10,*lines_4) 
        lines_d4 = zip(pd_00,pd_51,pd_52,pd_53,pd_54)
        sub.plot3D(color='#ff0789', linewidth=2,    marker='o',    
                mfc='#ff0789',   mec='#ff0789',   ms=4,*lines_d4)
        ## Fear
        lines_5 = zip(p_00,p_61, p_62,p_63,p_64)
        sub.plot3D(label='Fear', color='#2a7e19', linewidth=3,zdir='z',    marker='o',    
                mfc='#2a7e19',   mec='#2a7e19',   ms=10,*lines_5) 
        lines_d5 = zip(pd_00,pd_61, pd_62,pd_63,pd_64)
        sub.plot3D( color='#2a7e19', linewidth=2,    marker='o',    
                mfc='#2a7e19',   mec='#2a7e19',   ms=4,*lines_d5) 
        ## Disgust
        lines_6 = zip(p_00,p_71, p_72,p_73,p_74)
        sub.plot3D(label='Disgust', color='#a442a0', linewidth=3,zdir='z',    marker='o',    
                mfc='#a442a0',   mec='#a442a0',   ms=10,*lines_6)
        lines_d6 = zip(pd_00,pd_71, pd_72,pd_73,pd_74)
        sub.plot3D(color='#a442a0', linewidth=2,    marker='o',    
                mfc='#a442a0',   mec='#a442a0',   ms=4,*lines_d6) 
        ## Boring
        lines_7 = zip(p_00,p_81, p_82,p_83,p_84)
        sub.plot3D(label='Boring', color='#02ccfe', linewidth=3,zdir='z',    marker='o',    
                mfc='#02ccfe',   mec='#02ccfe',   ms=10,*lines_7)
        lines_d7 = zip(pd_00,pd_81, pd_82,pd_83,pd_84)
        sub.plot3D( color='#02ccfe', linewidth=2,    marker='o',    
                mfc='#02ccfe',   mec='#02ccfe',   ms=4,*lines_d7)  
        ## 打标签
        zdir = ('Joy', 'Trust', 'Antipation' ,'Sadness', 'Anger', 'Fear', 'Disgust', 'Boring')
        point_list=[p_14,p_24,p_34,p_44,p_54,p_64,p_74,p_84]
        for i in range(len(point_list)):
                sub.text(point_list[i][0], point_list[i][1],point_list[i][2] , r'$%s$'%zdir[i])
        plt.legend(loc=(0.8,0.5)) # 添加图例
        


        ### 生成图
        #sub.scatter(x,y,z,s=50) # 生成控制点集合
        sub.scatter(p_00[0],p_00[1],p_00[2],s=100,c='grey') # 修改中心点
        sub.scatter(pd_00[0],pd_00[1],pd_00[2],s=40,c='grey')
        trisurf=sub.plot_trisurf(x,y,z,color='grey',antialiased=False,alpha=0.35) # 生成网格面
        #fig.colorbar(trisurf, shrink=0.5, aspect=5)#加上色条

        ### 生成移动点
        move_point=[]
        for i in range(len(current_e)):
                if i == 0: # Joy
                        x=0;y=current_e[i]
                        point_clist= [p_00,p_11,p_12,p_13,p_14]
                        threshold = [p_00[1],p_11[1],p_12[1],p_13[1],p_14[1]]
                        h = bisect_left(threshold,y)
                        a,b,c=getLinearEquation(threshold[int(h-1)],point_clist[int(h-1)][2],threshold[int(h)],point_clist[int(h)][2])
                        z=(-a*y-c)/b
                        sub.scatter(x,y,z,s=100,color='#fcb001',edgecolors='#000000',linewidth=3)
                        sub.scatter(x,y,-0.3,s=40,color='#fcb001',edgecolors='#000000',linewidth=1)
                if i == 3: # Sadness
                        x=0;y=current_e[i]
                        point_clist= [p_00,p_41,p_42,p_43,p_44]
                        threshold = [-p_00[1],-p_41[1],-p_42[1],-p_43[1],-p_44[1]]
                        h = bisect_left(threshold,y)
                        a,b,c=getLinearEquation(threshold[int(h-1)],point_clist[int(h-1)][2],threshold[int(h)],point_clist[int(h)][2])
                        z=(-a*y-c)/b
                        sub.scatter(x,-y,z,s=100,color='#0e87cc',edgecolors='#000000',linewidth=3)
                        sub.scatter(x,-y,-0.3,s=40,color='#0e87cc',edgecolors='#000000',linewidth=1)
                if i == 4: # Anger
                        y=0;x=current_e[i]
                        point_clist= [p_00,p_51,p_52,p_53,p_54]
                        threshold = [-p_00[0],-p_51[0],-p_52[0],-p_53[0],-p_54[0]]
                        h = bisect_left(threshold,x)
                        a,b,c=getLinearEquation(threshold[int(h-1)],point_clist[int(h-1)][2],threshold[int(h)],point_clist[int(h)][2])
                        z=(-a*x-c)/b
                        sub.scatter(-x,y,z,s=100,color='#ff0789',edgecolors='#000000',linewidth=3)
                        sub.scatter(-x,y,-0.3,s=40,color='#ff0789',edgecolors='#000000',linewidth=1)
                if i == 5: # Fear
                        y=0;x=current_e[i]
                        point_clist= [p_00,p_61,p_62,p_63,p_64]
                        threshold = [p_00[0],p_61[0],p_62[0],p_63[0],p_64[0]]
                        h = bisect_left(threshold,x)
                        a,b,c=getLinearEquation(threshold[int(h-1)],point_clist[int(h-1)][2],threshold[int(h)],point_clist[int(h)][2])
                        z=(-a*x-c)/b
                        sub.scatter(x,y,z,s=100,color='#2a7e19',edgecolors='#000000',linewidth=3)
                        sub.scatter(x,y,-0.3,s=40,color='#2a7e19',edgecolors='#000000',linewidth=1)
                if i == 1: # Trust
                        x=current_e[i]/math.sqrt(2);y=x
                        point_clist= [p_00,p_21,p_22,p_23,p_24]
                        threshold = [p_00[0],p_21[0],p_22[0],p_23[0],p_24[0]]
                        h = bisect_left(threshold,x)
                        a,b,c=getLinearEquation(threshold[int(h-1)],point_clist[int(h-1)][2],threshold[int(h)],point_clist[int(h)][2])
                        z=(-a*x-c)/b
                        sub.scatter(x,y,z,s=100,color='#9ce143',edgecolors='#000000',linewidth=3)
                        sub.scatter(x,y,-0.3,s=40,color='#9ce143',edgecolors='#000000',linewidth=1)
                if i == 7: # Boring
                        x=current_e[i]/math.sqrt(2);y=-x
                        point_clist= [p_00,p_81,p_82,p_83,p_84]
                        threshold = [p_00[0],p_81[0],p_82[0],p_83[0],p_84[0]]
                        h = bisect_left(threshold,x)
                        a,b,c=getLinearEquation(threshold[int(h)-1],point_clist[int(h)-1][2],threshold[int(h)],point_clist[int(h)][2])
                        z=(-a*x-c)/b
                        sub.scatter(x,y,z,s=100,color='#02ccfe',edgecolors='#000000',linewidth=3)
                        sub.scatter(x,y,-0.3,s=40,color='#02ccfe',edgecolors='#000000',linewidth=1)
                if i == 6: # Disgust
                        x=current_e[i]/math.sqrt(2);y=x
                        point_clist= [p_00,p_71,p_72,p_73,p_74]
                        threshold = [-p_00[0],-p_71[0],-p_72[0],-p_73[0],-p_74[0]]
                        h = bisect_left(threshold,x)
                        a,b,c=getLinearEquation(threshold[int(h-1)],point_clist[int(h-1)][2],threshold[int(h)],point_clist[int(h)][2])
                        z=(-a*x-c)/b
                        sub.scatter(-x,-y,z,s=100,color='#a442a0',edgecolors='#000000',linewidth=3)
                        sub.scatter(-x,-y,-0.3,s=40,color='#a442a0',edgecolors='#000000',linewidth=1)
                if i == 2: # Anticipation
                        y=current_e[i]/math.sqrt(2);x=y
                        point_clist= [p_00,p_31,p_32,p_33,p_34]
                        threshold = [p_00[1],p_31[1],p_32[1],p_33[1],p_34[1]]
                        h = bisect_left(threshold,x)
                        a,b,c=getLinearEquation(threshold[int(h-1)],point_clist[int(h-1)][2],threshold[int(h)],point_clist[int(h)][2])
                        z=(-a*x-c)/b
                        sub.scatter(-x,y,z,s=100,color='#ff5b00',edgecolors='#000000',linewidth=3)
                        sub.scatter(-x,y,-0.3,s=40,color='#ff5b00',edgecolors='#000000',linewidth=1)


        ### 调整视角
        sub.view_init(elev=24,    # 仰角
                azim=-103)   # 方位角
        # print("Here is the visual value:",current_e)
        # plt.show()
        # plt.draw()
        plt.savefig(os.path.join(root, 'image/test_boring/'+str(ii)+'test.png'))
        # plt.pause(0.02)
        # plt.ioff()
        
def read_goal():
        temp_lst = []
        with open(os.path.join(root, 'scripts/csv/test_boring.csv'), 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                        temp_lst.append(row)
        return temp_lst

if __name__ == '__main__':
#         # # current_e=[0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4]
#         # # current_e=[0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]
        # current_e=[0.6,0.6,0.6,0.6,0.6,0.4,0.6,0.6]
        # current_e=[0,0,0,0,0,0,0,0]
        # plt.ion() 

        
        current_elst=read_goal()
        fig = plt.figure(figsize=(9.6, 8.4)) # 图像像素大小为960*840
        for i in range(1,len(current_elst)):
                print([float(j) for j in current_elst[i]])
                update_visual([float(j) for j in current_elst[i]],fig,i)
        
        # current_e=[0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4]
        # elev_e=24
        # azim_e=-103
        # fig = plt.figure(figsize=(9.6, 8.4)) # 图像像素大小为960*840
        # update_visual(current_e,fig)