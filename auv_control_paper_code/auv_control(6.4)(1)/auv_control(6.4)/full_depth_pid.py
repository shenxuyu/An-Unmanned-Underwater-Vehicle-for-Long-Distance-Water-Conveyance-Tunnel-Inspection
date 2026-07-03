#!/usr/bin/env python3
#coding: utf-8 

import time

class FullDepthPID:
    def __init__(self, Kp, Ki, Kd, sample_time=0.00):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.sample_time = sample_time
        self.current_time = time.time()
        self.last_time = self.current_time
        self.current_depth = 0.0
        self.ITerm = 0
        self.clear()

    def clear(self):
        self.setdepth = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.windup_guard = 80
        self.output = 0.0

    def update(self, feedback_depth, feedback_dvl_x):
        error = self.setdepth - feedback_depth
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
        if delta_time >= self.sample_time:
            if delta_time >= self.sample_time:  # 如果超过采样时间
                if feedback_dvl_x > 0.8:
                    self.PTerm = 2 * self.Kp * error * feedback_dvl_x ** 2  # 计算比例项
                else:
                    self.PTerm = self.Kp * error
            # self.PTerm = self.Kp * error  # 比例项
            if abs(error) <= 0.25:
                self.ITerm += self.Ki * error * delta_time  # 积分项
                # 保护
                if self.ITerm < -1*self.windup_guard:
                    self.ITerm = -1*self.windup_guard
                elif self.ITerm > self.windup_guard:
                    self.ITerm = self.windup_guard
            else:
                self.ITerm = 0
            # self.DTerm = 0.0
            if delta_time > 0:
                # print("D项判断进入")
                # print("delta_error")
                # print(delta_error)
                # print("delta_time")
                # print(delta_time)
                self.DTerm = self.Kd * (delta_error / delta_time)  # 微分项
            self.last_time = self.current_time
            self.last_error = error
            self.output = self.PTerm + self.ITerm + self.DTerm

    def setKp(self,kp):
        self.Kp = kp
    
    def setKi(self,ki):
        self.Ki = ki

    def setKd(self, kd):
        self.Kd = kd

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSanmpleTime(self, sample_time):
        self.sample_time = sample_time

    def set_depth(self, exp_depth):
        self.setdepth = exp_depth