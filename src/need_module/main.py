'''
Descripttion: 
version: 
Author: Zhang Jiadong
Date: 2022-03-29 10:32:43
LastEditors: Zhang Jiadong
LastEditTime: 2022-03-29 10:52:15
'''

#!/usr/bin/env python
#encoding=utf-8
#coding=utf-8
filename = '/home/zhjd/ws/src/social_system/need_module/test.xlsx'

import xlrd
book=xlrd.open_workbook(filename) # 打开要读取的Excel
sheet=book.sheet_by_name('school')    # 打开xiangxin页
rows=sheet.nrows    # xiangxin页里面的行数
columns=sheet.ncols # xiangxin页里面的列数
print('行数为: ',rows)
print('列数为: ',columns)
print(sheet.cell(1,2).value)    # 通过制定行和列去获取到单元格里的内容

row_data=sheet.row_values(2)    # 获取第三行的内容,返回的是一个列表
print(row_data)

for i in range(rows):
    print(sheet.row_values(i))  # 遍历所有行的数据,返回的是列表数据类型