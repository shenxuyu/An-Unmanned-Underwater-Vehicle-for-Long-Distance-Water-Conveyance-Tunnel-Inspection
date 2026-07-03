#!/usr/bin/env python3
# -*- coding:UTF-8 -*-

# author: Yu xing
# contact: 17824845379@163.com
# datetime:2023/10/6 18:40
# software: PyCharm
# 引入第三方模糊控制系统库
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt


class FuzzyCtl:
    def __init__(self):
        # 输出的规则
        self.rule1 = None
        self.rule2 = None
        self.rule3 = None
        self.rule4 = None
        self.rule5 = None
        self.rule6 = None
        self.rule7 = None
        # e 和 ec的输入
        self.e_input = 0
        self.ec_input = 0
        # 模糊化
        # 1.论域的确定
        # 设置偏差、偏差率，输出的论域并采用默认模式
        # 设置变量范围
        self.e_range = np.arange(-1, 1.2, 0.2)   # [-1,1]
        self.ec_range = np.arange(-1, 1.2, 0.2)  # [-1,1]
        self.fuzzy_output_range = np.arange(-1, 1.2, 0.2)  # [-1,1]
        # 创建模糊控制变量
        self.e = ctrl.Antecedent(self.e_range, 'e')  # 输入1-偏差[0,10]
        self.ec = ctrl.Antecedent(self.ec_range, 'ec')  # 输入2-偏差率[0,10]
        self.fuzzy_output = ctrl.Consequent(self.fuzzy_output_range, 'fuzzy_output')  # 输出
        self.get_membership()
        # 创建视图并显示
        ###########################################
        # self.e.view()
        # plt.show()
        # self.ec.view()
        # plt.show()
        # self.fuzzy_output.view()
        # plt.show()
        ########################################
        self.ctl_rules()  # 编辑规则集
        # 设定输出的解模糊方法——质心解模糊方式
        self.fuzzy_output.defuzzify_method = 'centroid'
        # 控制系统设置输入规则集
        self.fuzzy_output_ctrl = ctrl.ControlSystem([
            self.rule1, self.rule2, self.rule3, self.rule4,
            self.rule5, self.rule6, self.rule7])
        # 模拟控制系统
        self.fuzzy_outputting = ctrl.ControlSystemSimulation(self.fuzzy_output_ctrl)
        # 输入 e 与 ec
        # 将俯仰偏差e[-30, 30]映射到[-1, 1]
        e_norm = self.normalizing(-1, 1, -30, 30, self.e_input)
        self.fuzzy_outputting.input['e'] = e_norm
        # 将俯仰偏差变化率ec[-100, 100]映射到[-1, 1]
        ec_norm = self.normalizing(-1, 1, -100, 100, self.ec_input)
        self.fuzzy_outputting.input['ec'] = ec_norm
        # 计算
        self.fuzzy_outputting.compute()
        # 打印模糊控制后的输出，即kp
        self.fuzzy_output_kp = self.fuzzy_outputting.output['fuzzy_output']
        # 打印输出结果
        # 画图显示结果输出
        # print(self.fuzzy_output_kp)
        # self.fuzzy_output.view(sim=self.fuzzy_outputting)
        # plt.show()

    def get_membership(self):
        """
        2.隶属度函数的确定,一般使用三角形隶属度函数，
        可以自己定义隶属度范围，也可以用automf函数自动生成
        """
        # 定义偏差差时的三角隶属度函数横坐标
        # self.e.automf(7)  # 设置7个参考值，自动生成
        self.e['NB'] = fuzz.trimf(self.e_range, [-1000, -0.7, -0.5])
        self.e['NM'] = fuzz.trimf(self.e_range, [-0.6, -0.4, -0.2])
        self.e['NS'] = fuzz.trimf(self.e_range, [-0.4, -0.2, 0])
        self.e['ZE'] = fuzz.trimf(self.e_range, [-0.1, 0, 0.1])
        self.e['PS'] = fuzz.trimf(self.e_range, [0, 0.2, 0.4])
        self.e['PM'] = fuzz.trimf(self.e_range, [0.2, 0.4, 0.6])
        self.e['PB'] = fuzz.trimf(self.e_range, [0.5, 0.7, 1000])
        # 定义偏差率差时的三角隶属度函数横坐标
        self.ec['NB'] = fuzz.trimf(self.ec_range, [-1000, -0.7, -0.5])
        self.ec['NM'] = fuzz.trimf(self.ec_range, [-0.6, -0.4, -0.2])
        self.ec['NS'] = fuzz.trimf(self.ec_range, [-0.4, -0.2, 0])
        self.ec['ZE'] = fuzz.trimf(self.ec_range, [-0.1, 0, 0.1])
        self.ec['PS'] = fuzz.trimf(self.ec_range, [0, 0.2, 0.4])
        self.ec['PM'] = fuzz.trimf(self.ec_range, [0.2, 0.4, 0.6])
        self.ec['PB'] = fuzz.trimf(self.ec_range, [0.5, 0.7, 1000])
        # 定义输出差时的三角隶属度函数横坐标
        # 输出设置7个参考值
        self.fuzzy_output['NB'] = fuzz.trimf(self.fuzzy_output_range, [-1, -0.8, -0.4])
        self.fuzzy_output['NM'] = fuzz.trimf(self.fuzzy_output_range, [-0.6, -0.4, -0.2])
        self.fuzzy_output['NS'] = fuzz.trimf(self.fuzzy_output_range, [-0.4, -0.2, 0])
        self.fuzzy_output['ZE'] = fuzz.trimf(self.fuzzy_output_range, [-0.1, 0, 0.1])
        self.fuzzy_output['PS'] = fuzz.trimf(self.fuzzy_output_range, [0, 0.2, 0.4])
        self.fuzzy_output['PM'] = fuzz.trimf(self.fuzzy_output_range, [0.2, 0.4, 0.6])
        self.fuzzy_output['PB'] = fuzz.trimf(self.fuzzy_output_range, [0.4, 0.8, 1])

    def ctl_rules(self):
        # 输出为NB的规则
        self.rule1 = ctrl.Rule(
            antecedent=((self.e['PB'] & self.ec['PM']) |
                        (self.e['PM'] & self.ec['PB']) |
                        (self.e['PB'] & self.ec['PB'])),
            consequent=self.fuzzy_output['NB'], label='rule NB')
        # 可视化规则
        # self.rule1.view()
        # plt.show()
        # 输出为NM的规则
        self.rule2 = ctrl.Rule(
            antecedent=((self.e['PB'] & self.ec['ZE']) |
                        (self.e['PM'] & self.ec['PS']) |
                        (self.e['PB'] & self.ec['PS']) |
                        (self.e['PS'] & self.ec['PM']) |
                        (self.e['PM'] & self.ec['PM']) |
                        (self.e['ZE'] & self.ec['PB']) |
                        (self.e['PB'] & self.ec['PB'])),
            consequent=self.fuzzy_output['NM'], label='rule NM')
        # 输出为NS的规则
        self.rule3 = ctrl.Rule(
            antecedent=((self.e['PB'] & self.ec['NM']) |
                        (self.e['PM'] & self.ec['NS']) |
                        (self.e['PB'] & self.ec['NS']) |
                        (self.e['PS'] & self.ec['ZE']) |
                        (self.e['PM'] & self.ec['ZE']) |
                        (self.e['ZE'] & self.ec['PS']) |
                        (self.e['PS'] & self.ec['PS']) |
                        (self.e['NS'] & self.ec['PM']) |
                        (self.e['ZE'] & self.ec['PM']) |
                        (self.e['NM'] & self.ec['PB']) |
                        (self.e['NS'] & self.ec['PB'])),
            consequent=self.fuzzy_output['NS'], label='rule NS')
        # 输出为ZE的规则
        self.rule4 = ctrl.Rule(
            antecedent=((self.e['PB'] & self.ec['NB']) |
                        (self.e['PM'] & self.ec['NM']) |
                        (self.e['PS'] & self.ec['NS']) |
                        (self.e['ZE'] & self.ec['ZE']) |
                        (self.e['NS'] & self.ec['PS']) |
                        (self.e['NM'] & self.ec['PM']) |
                        (self.e['NB'] & self.ec['PB'])),
            consequent=self.fuzzy_output['ZE'], label='rule ZE')
        # 输出为PS的规则
        self.rule5 = ctrl.Rule(
            antecedent=((self.e['PS'] & self.ec['NB']) |
                        (self.e['PM'] & self.ec['NB']) |
                        (self.e['ZE'] & self.ec['NM']) |
                        (self.e['PS'] & self.ec['NM']) |
                        (self.e['NS'] & self.ec['NS']) |
                        (self.e['ZE'] & self.ec['NS']) |
                        (self.e['NM'] & self.ec['ZE']) |
                        (self.e['NS'] & self.ec['ZE']) |
                        (self.e['NB'] & self.ec['PS']) |
                        (self.e['NM'] & self.ec['PS']) |
                        (self.e['NB'] & self.ec['PM'])),
            consequent=self.fuzzy_output['PS'], label='rule PS')
        # 输出为PM的规则
        self.rule6 = ctrl.Rule(
            antecedent=((self.e['NS'] & self.ec['NB']) |
                        (self.e['ZE'] & self.ec['NB']) |
                        (self.e['NM'] & self.ec['NM']) |
                        (self.e['NS'] & self.ec['NM']) |
                        (self.e['NB'] & self.ec['NS']) |
                        (self.e['NM'] & self.ec['NS']) |
                        (self.e['NB'] & self.ec['ZE'])),
            consequent=self.fuzzy_output['PM'], label='rule PM')
        # 输出为PB的规则
        self.rule7 = ctrl.Rule(
            antecedent=((self.e['NB'] & self.ec['NB']) |
                        (self.e['NM'] & self.ec['NB']) |
                        (self.e['NB'] & self.ec['NM'])),
            consequent=self.fuzzy_output['PB'], label='rule PB')

    # 映射函数
    def normalizing(self, x_min, x_max, a, b, x, ):
        """
        x_min,x_max 是输入值的范围[x_min,x_max]
        a,b 是归一化到[a,b]区间
        x是输入
        """
        y_out = (b - a) * (x - x_min) / (x_max - x_min) + a
        return y_out

    def set_e(self, e):
        self.e_input = e

    def set_ec(self, ec):
        self.ec_input = ec


if __name__ == '__main__':
    FC = FuzzyCtl()


