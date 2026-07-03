#!/usr/bin/env python3
# -*- coding:UTF-8 -*-

# author: Yu xing
# contact: 17824845379@163.com
# datetime:2023/10/6 9:57
# software: PyCharm

import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt
from skfuzzy import control as ctrl
import math

#  质量和服务范围为[0，10]
#  小费范围为[0，25]
x_qual = np.arange(0, 11, 1)
x_serv = np.arange(0, 11, 1)
x_tip = np.arange(0, 26, 1)
# 定义模糊控制变量
quality = ctrl.Antecedent(x_qual, 'quality')
service = ctrl.Antecedent(x_serv, 'service')
tip = ctrl.Consequent(x_tip, 'tip')
# 生成模糊隶属函数
quality['L'] = fuzz.trimf(x_qual, [0, 0, 5])  # 定义质量差时的三角隶属度函数横坐标
quality['M'] = fuzz.trimf(x_qual, [0, 5, 10])
quality['H'] = fuzz.trimf(x_qual, [5, 10, 10])
service['L'] = fuzz.trimf(x_serv, [0, 0, 5])  # 定义服务差时的三角隶属度函数横坐标
service['M'] = fuzz.trimf(x_serv, [0, 5, 10])
service['H'] = fuzz.trimf(x_serv, [5, 10, 10])
tip['L'] = fuzz.trimf(x_tip, [0, 0, 13])  # 定义小费的三角隶属度函数横坐标
tip['M'] = fuzz.trimf(x_tip, [0, 13, 25])
tip['H'] = fuzz.trimf(x_tip, [13, 25, 25])

tip.defuzzify_method = 'centroid'
# 可视化这些输入输出和隶属函数
# quality.automf(3)
# service.automf(3)#三种程度
# quality.view()
# service.view()
# plt.show()
# 规则
rule1 = ctrl.Rule(
    antecedent=((quality['L'] & service['L']) | (quality['L'] & service['M']) | (quality['M'] & service['L'])),
    consequent=tip['L'], label='Low')
rule2 = ctrl.Rule(
    antecedent=((quality['M'] & service['M']) | (quality['L'] & service['H']) | (quality['H'] & service['L'])),
    consequent=tip['M'], label='Medium')
rule3 = ctrl.Rule(
    antecedent=((quality['M'] & service['H']) | (quality['H'] & service['M']) | (quality['H'] & service['H'])),
    consequent=tip['H'], label='High')

rule2.view()
tipping_ctrl = ctrl.ControlSystem([rule1, rule2, rule3])
tipping = ctrl.ControlSystemSimulation(tipping_ctrl)
# 测试输出
# tipping.input['quality'] = 6.5
# tipping.input['service'] = 9.8
# tipping.compute()
# print(tipping.output['tip'])
# tip.view(sim=tipping)
# plt.show()

# 仿真结果3D图输出，使用下列代码时请注释掉上面的测试输出
upsampled = np.linspace(0, 11, 21)  # 这里的范围不能错
x, y = np.meshgrid(upsampled, upsampled)
z = np.zeros_like(x)
##tipping.input['angle'] = 0
##tipping.input['distance'] = 0
##tipping.compute()
##print (tipping.output['out'])
# out.view(sim=tipping)
pp = []
for i in range(0, 21):
    for j in range(0, 21):
        tipping.input['quality'] = x[i, j]
        tipping.input['service'] = y[i, j]
        tipping.compute()
        z[i, j] = tipping.output['tip']
        pp.append(z[i, j])
print('max:', max(pp))
print('min:', min(pp))

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure(figsize=(8, 8))  # 定义画布大小
ax = fig.add_subplot(111, projection='3d')
surf = ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap='viridis', linewidth=0.4, antialiased=True)
# cset = ax.contourf(x, y, z, zdir='z', offset=-2.5, cmap='viridis', alpha=0.5)
# cset = ax.contourf(x, y, z, zdir='x', offset=3, cmap='viridis', alpha=0.5)
# cset = ax.contourf(x, y, z, zdir='y', offset=3, cmap='viridis', alpha=0.5)
ax.view_init(30, 200)  # 设置观察角度
plt.show()
