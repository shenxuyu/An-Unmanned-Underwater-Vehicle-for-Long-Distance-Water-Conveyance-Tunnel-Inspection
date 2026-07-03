# 编辑人:努力学习的小李
# 开发时间:2024/1/4 11:16
# 座右铭:今天不学习，明天变废物！
# !/usr/bin/env python3
# coding: utf-8
import time
import math


class UnderDepthBackStep:
    def __init__(self, K1, K2, K3, sample_time=0.00):
        # 反步法控制参数
        self.K1 = K1
        self.K2 = K2
        self.K3 = K3
        self.b = 0.1774
        self.last_delta_h = 0.0
        self.last_theta = 0
        self.last_delta_theta = 0
        self.theta_d = 0
        self.sample_time = sample_time
        self.current_time = time.time()
        self.last_time = self.current_time
        self.current_depth = 0

        #$$$$$$$$$$$$$$$$$$LC add$$$$$$$$$$$$$$$
        # 纵倾角速度
        self.pitch_speed = 0
        # 期望纵倾角速度
        self.pitch_speed_d = 0

        self.output = 0
        self.exp_depth = 0.0
        self.clear()

    def clear(self):
        self.exp_depth = 0.0
        self.output = 0.0
        self.last_delta_h = 0.0
        self.last_theta = 0
        self.last_delta_theta = 0
        self.theta_d = 0
        self.current_depth = 0

    def update(self, feedback_depth, feedback_pitch, feedback_speed):
        # 将反馈俯仰角转为弧度
        feedback_pitch = math.radians(feedback_pitch)
        # 深度误差；
        delta_h = feedback_depth - self.exp_depth
        # # 保护
        # if abs(delta_h) >= 1:
        #     delta_h = 1
        # # 调试用时间
        # time.sleep(2)
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        # print("delta_time=", delta_time)
        if delta_time >= self.sample_time and feedback_speed > 0.004:
            # 期望纵倾角计算
            self.theta_d = self.K1 * delta_h / feedback_speed
            # 纵倾角误差
            delta_theta = feedback_pitch - self.theta_d
            # 计算期望纵倾角速度
            self.pitch_speed_d = -self.K1 * self.K1 * delta_h / feedback_speed - self.K1 * delta_theta \
                                -self.K2 * delta_theta
            # 计算纵倾角速度
            self.pitch_speed = (feedback_pitch - self.last_theta) / delta_time
            # 计算纵倾角速度误差
            delta_theta_speed = self.pitch_speed - self.pitch_speed_d

            #---------------------------------------------------------------------------------
            # 深度误差变化量
            d_delta_h = delta_h - self.last_delta_h
            # 纵倾角误差变化量
            d_delta_theta = delta_theta - self.last_delta_theta
            # 深度误差对时间的导数
            dd_delta_h = d_delta_h / delta_time
            # 纵倾角误差对时间的导数
            dd_delta_theta = d_delta_theta / delta_time
            # ---------------------------------------------------------------------------------

            #--------------反步法控制律为-k3*feedback_speed*delta_theta_speed--------------------
            self.output = -self.K3 * feedback_speed * delta_theta_speed / self.b
            # self.output = -(self.K1 * self.K1 * dd_delta_h + self.K1 * dd_delta_theta \
            #                 + self.K2 * dd_delta_theta + self.K3 * delta_theta_speed)
            #---------------------------------------------------------------------------------
            self.last_delta_h = delta_h
            self.last_delta_theta = delta_theta
            self.last_time = self.current_time
            print("计算期望纵倾角,当前纵倾角,反步法输出", math.degrees(self.theta_d), math.degrees(feedback_pitch), self.output)

    def setb(self, b):
        self.b = b

    def setK1(self, k1):
        self.K1 = k1

    def setK2(self, k2):
        self.K2 = k2

    def setK3(self, k3):
        self.K3 = k3


    def setSanmpleTime(self, sample_time):
        self.sample_time = sample_time

    def set_depth(self, exp_depth):
        self.exp_depth = exp_depth

# if __name__ == '__main__':
#
#     set_depth = 1
#     depth_data = 0.9
#     print('--------------------------------------------------------')
#     print("深度误差：", set_depth-depth_data)
#     pitch_data = 0
#     print("当前倾角：", pitch_data)
#     current_speed = 3.5
#     print("当前速度：", current_speed)
#     depth_calculate = UnderDepthBackStep(K1=1, K2=2, K3=2, sample_time=0.1)
#     depth_calculate.set_depth(set_depth)
#     depth_calculate.update(depth_data, pitch_data,current_speed)
#     theta = depth_calculate.output
#     print("计算得到角度：", theta)
#     if theta <= -45:
#         theta = -45
#     elif theta >= 45:
#         theta = 45
#     print('定深输出theta：', theta)
#     expect_theta_speed = depth_calculate.pitch_speed_d
#     print("计算得到期望纵倾角速度：",expect_theta_speed)


