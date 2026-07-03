#!/usr/bin/env python3
# coding: utf-8
import time
from fuzzy_ctl import FuzzyCtl as FC


class UnderDepthPID:
    def __init__(self, Kp, Ki, Kd, sample_time=0.00):
        # 纵倾角计算PID参数
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        # 深度误差计算p项参数
        self.k = -3.0
        # 深度误差计算I项参数
        self.i = -10.0
        self.last_delta_h = 0.0
        self.last_theta = 0
        self.last_delta_theta = 0
        self.theta_d = 0
        self.sample_time = sample_time
        self.current_time = time.time()
        self.last_time = self.current_time
        self.current_depth = 0
        self.PTerm1 = 0
        self.ITerm1 = 0
        self.PTerm = 0
        self.ITerm = 0
        self.DTerm = 0
        self.last_error = 0
        self.output = 0
        self.exp_depth = 0.0
        # 用于保存数据的
        #######################
        self.e_pitch_save = 0
        self.ec_pitch_save = 0
        #######################
        self.clear()
        self.fuzzy_res_kp = 0  # 用于接收模糊后的kp结果
        self.FC = FC()
        # 用于保存数据的
        self.e_pitch_save = 0
        self.ec_pitch_save = 0

    def clear(self):
        self.exp_depth = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.windup_guard = 80
        self.output = 0.0
        self.PTerm1 = 0.0
        self.ITerm1 = 0.0

    def update(self, feedback_depth, feedback_pitch):
        # 深度误差；
        delta_h = self.exp_depth - feedback_depth
        # 保护
        if abs(delta_h) >= 1:
            delta_h = 1
        # 调试用时间
        # time.sleep(1)
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        # print("delta_time=", delta_time)
        # 深度P控制器---外环
        if delta_time >= self.sample_time:
            self.PTerm1 = self.k * delta_h  # 比例项
            # if abs(delta_h) <= 0.25:
            #     self.ITerm1 += self.i * delta_h * delta_time  # 积分项
            #     # 保护
            #     if self.ITerm1 < -1*self.windup_guard:
            #         self.ITerm1 = -1*self.windup_guard
            #     elif self.ITerm1 > self.windup_guard:
            #         self.ITerm1 = self.windup_guard
            # 深度已转为纵倾角
            self.theta_d = self.PTerm1  # + self.ITerm1
        # 纵倾角PD控制器----内环
        # 纵倾角误差
            delta_theta = self.theta_d - feedback_pitch
            # 纵倾角误差变化量
            d_delta_theta = delta_theta - self.last_delta_theta
            # 加入模糊控制器
            # =====================================================
            e = delta_theta
            ec = d_delta_theta / delta_time
            # 用于保存数据的
            #########################
            self.e_pitch_save = e
            self.ec_pitch_save = ec
            #####################
            self.FC.set_e(e)
            self.FC.set_ec(ec)
            self.fuzzy_res_kp = self.FC.fuzzy_output_kp
            # =====================================================
        # if delta_time >= self.sample_time:
            # 输出值为[-1,1]，按照PID控制范围加入适当增益
            self.PTerm = (1 * self.fuzzy_res_kp + self.Kp) * delta_theta
            if abs(delta_theta) <= 5:
                self.ITerm += self.Ki * delta_theta * delta_time
                if self.ITerm < -1*self.windup_guard:
                    self.ITerm = -1*self.windup_guard
                elif self.ITerm > self.windup_guard:
                    self.ITerm = self.windup_guard
            else:
                self.ITerm = 0
            # self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = self.Kd * (d_delta_theta / delta_time)
            self.last_error = delta_theta
            self.output = self.PTerm + self.ITerm + self.DTerm
            self.last_time = self.current_time
            self.last_delta_theta = d_delta_theta
            print("计算纵倾角,当前纵倾角,PID输出", self.theta_d, feedback_pitch, self.output)

    def setK(self, k):
        self.k = k
        
    def setK(self, d):
        self.k = d

    def setKp(self, kp):
        self.Kp = kp

    def setKi(self, ki):
        self.Ki = ki

    def setKd(self, kd):
        self.Kd = kd

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSanmpleTime(self, sample_time):
        self.sample_time = sample_time

    def set_depth(self, exp_depth):
        self.exp_depth = exp_depth


# if __name__ == '__main__':
#
#     set_depth = 2.8
#     depth_data = 2.8
#     print('--------------------------------------------------------')
#     print("深度误差：", set_depth-depth_data)
#     pitch_data = -5
#     print("当前倾角：", pitch_data)
#     depth_calculate = UnderDepthPID(Kp=5, Ki=0, Kd=0, sample_time=0.1)
#     depth_calculate.set_depth(set_depth)
#     depth_calculate.update(depth_data, pitch_data)
#     theta = depth_calculate.output
#     print("计算得到角度：", theta)
#     if theta <= -45:
#         theta = -45
#     elif theta >= 45:
#         theta = 45
#     print('定深输出theta：', theta)