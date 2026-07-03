#!/usr/bin/env python3
# -*- coding:UTF-8 -*-

# author: Yu xing
# contact: 17824845379@163.com
# datetime:2023/10/6 10:14
# software: PyCharm

# 引入第三方模糊控制系统库
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

# 设置污泥，油脂，洗涤时间的论域并采用默认模式
stain = ctrl.Antecedent(np.arange(0, 101, 1), 'stain')
oil = ctrl.Antecedent(np.arange(0, 101, 1), 'oil')
washtime = ctrl.Consequent(np.arange(0, 121, 1), 'washtime')

# 自动分析模糊函数
stain.automf(3, variable_type='quant')
oil.automf(3, variable_type='quant')

# 设置洗涤时间依据污泥，油脂进行的分类标准
washtime['VS'] = fuzz.trimf(washtime.universe, [0, 0, 20])
washtime['S'] = fuzz.trimf(washtime.universe, [0, 20, 50])
washtime['M'] = fuzz.trimf(washtime.universe, [20, 50, 70])
washtime['L'] = fuzz.trimf(washtime.universe, [50, 70, 100])
washtime['VL'] = fuzz.trimf(washtime.universe, [70, 100, 120])

# 创建视图，并展示视图
stain['average'].view()
plt.show()

oil.view()
plt.show()

washtime.view()
plt.show()

# 模糊控制规则表
rule1 = ctrl.Rule(stain['low'] & oil['low'], washtime['VS'])
rule2 = ctrl.Rule(stain['low'] & oil['average'], washtime['M'])
rule3 = ctrl.Rule(stain['low'] & oil['high'], washtime['L'])
rule4 = ctrl.Rule(stain['average'] & oil['low'], washtime['S'])
rule5 = ctrl.Rule(stain['average'] & oil['average'], washtime['M'])
rule6 = ctrl.Rule(stain['average'] & oil['high'], washtime['L'])
rule7 = ctrl.Rule(stain['high'] & oil['low'], washtime['M'])
rule8 = ctrl.Rule(stain['high'] & oil['average'], washtime['L'])
rule9 = ctrl.Rule(stain['high'] & oil['high'], washtime['VL'])

# 控制系统设置输入规则集
washtimeping_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9])

# 模拟控制系统
washtimeping = ctrl.ControlSystemSimulation(washtimeping_ctrl)

# 输入污泥与油脂
washtimeping.input['stain'] = eval(input())
washtimeping.input['oil'] = eval(input())

# 计算洗涤时间
washtimeping.compute()

# 打印洗涤时间以及展示最终洗涤时间视图
print(washtimeping.output['washtime'])

washtime.view(sim=washtimeping)
plt.show()
