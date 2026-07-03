#!/usr/bin/env python3
# coding: utf-8

import time


# 舵机PID控制器类
class CoursePID:
    def __init__(self, Kp, Ki, Kd, sample_time=0.00):
        self.Kp = Kp          # 比例系数
        self.Ki = Ki          # 积分系数
        self.Kd = Kd          # 微分系数
        self.sample_time = sample_time   # 采样时间
        self.current_time = time.time()  # 当前时间
        self.last_time = self.current_time  # 上一次更新时间
        self.current_course = 0.0     # 当前位置（偏转角）
        self.exp_course = 0.0  # 目标位置
        self.PTerm = 0.0  # 比例项
        self.ITerm = 0.0  # 积分项
        self.DTerm = 0.0  # 微分项
        self.last_error = 0.0  # 上一次误差
        self.windup_guard = 0  # 防止积分值过大（积分饱和保护）
        self.output = 0.0  # PID输出值
        self.clear()       # 清空状态

    # 清空状态
    def clear(self):
        self.exp_course = 0.0     # 目标位置
        self.PTerm = 0.0         # 比例项
        self.ITerm = 0.0         # 积分项
        self.DTerm = 0.0         # 微分项
        self.last_error = 0.0    # 上一次误差
        self.windup_guard = 75   # 防止积分值过大（积分饱和保护）
        self.output = 0.0        # PID输出值

    # 更新PID控制器状态
    def update(self, feedback_course, feedback_dvl_x):
        error = self.exp_course - feedback_course   # 计算误差
        # 偏转角最小化处理
        if error > 180:
            error = error - 360
        elif error < -180:
            error = 360 + error
        self.current_time = time.time()  # 更新当前时间
        delta_time = self.current_time - self.last_time   # 计算采样时间
        delta_error = error - self.last_error             # 计算误差变化量
        if delta_time >= self.sample_time:                # 如果超过采样时间
            if feedback_dvl_x > 1:
                self.PTerm = 2.5 * self.Kp * error * feedback_dvl_x**2  # 计算比例项
            else:
                self.PTerm = self.Kp * error
            if abs(error) <= 10:                            # 如果误差<10度
                self.ITerm += self.Ki * error * delta_time   # 计算积分项
                if self.ITerm < -1*self.windup_guard:      # 积分项下限（防止饱和）
                    self.ITerm = -1*self.windup_guard
                elif self.ITerm > self.windup_guard:       # 积分项上限（防止饱和）
                    self.ITerm = self.windup_guard
            else:
                self.ITerm = 0
            self.DTerm = 0.0                               # 初始化微分项
            if delta_time > 0:                             # 如果采样时间大于0
                self.DTerm = self.Kd * (delta_error / delta_time)   # 计算微分项
            self.last_time = self.current_time             # 更新上一次更新时间
            self.last_error = error                        # 更新上一次误差
            self.output = self.PTerm + self.ITerm + self.DTerm    # 计算PID输出值

    # 设置比例系数
    def setKp(self, kp):
        self.Kp = kp
    
    # 设置积分系数
    def setKi(self, ki):
        self.Ki = ki

    # 设置微分系数
    def setKd(self, kd):
        self.Kd = kd

    # 设置防止饱和保护（积分项上下限）
    def setWindup(self, windup):
        self.windup_guard = windup

    # 设置采样时间
    def setSanmpleTime(self, sample_time):
        self.sample_time = sample_time

    # 设置目标位置
    def set_course(self, exp_course):
        self.exp_course = exp_course