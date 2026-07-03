#!/usr/bin/env python3
# coding: utf-8

import numpy as np
from numpy.linalg import pinv
import math


class ThrustDis:
    def __init__(self) -> None:
        self.distance_hor = 1.8  # 前后侧推之间的距离，单位m
        self.distance_ver = 2.0  # 前后垂推之间的距离，单位m
        self.Main_thrust = 400  # 主推最大推力40KG
        self.Lat_thrust = 65  # 侧、垂推最大推力6.5KG
        self.Main_speed = 2000  # 主推最大转速
        self.Lat_speed = 2000  # 侧、垂推最大转速
        self.thrust_list = []
        self.speed_list = []
        self.t_matrix = []
        self.set_matrix(self.distance_hor, self.distance_ver)

    def set_matrix(self, d1, d2):
        """
        推进器系统配置矩阵
        :param d1:
        :param d2:
        :return: 构造好的矩阵
        """
        row1 = np.array([1, 0, 0, 0, 0])
        row2 = np.array([0, -1, 1, 0, 0])
        row3 = np.array([0, 0, 0, 1, 1])
        row4 = np.array([0, -d1 / 2, -1 * d1 / 2, 0, 0])  # 左转 右转
        row5 = np.array([0, 0, 0, -1 * d2 / 2, d2 / 2])   # 纵倾
        self.t_matrix = np.array([row1, row2, row3, row4, row5])
        return self.t_matrix

    def thrust_distribute(self, x, y, z, n, m):
        """
        force_matrix = t_matrix * thrust_matrix
        :param x: auv前进方向的力矩
        :param y: auv左右移方向的力矩
        :param z: auv上下方向的力矩
        :param n: auv左右转方向的转矩
        :param m: auv俯仰方向的转矩
        :return: 各推进器速度
        """
        self.thrust_list.clear()
        self.speed_list.clear()
        pinv_matrix = pinv(self.t_matrix)  # pinv()求伪逆
        # 推进器输出的真实控制力矩
        force_matrix = np.matrix([x, y, z, n, m]).T
        # 求出各个推进器各需要输出多少的推力f
        thrust_matrix = pinv_matrix * force_matrix
        for thrust in thrust_matrix:
            item = round(float(thrust), 2)
            self.thrust_list.append(item)
        for i in range(len(self.thrust_list)):
            # main thruster:
            if i == 0:
                # 推力与速度的转化
                # 褚老师的方法，前面力量大，后面力量小
                speed = round(math.sqrt(abs(self.thrust_list[i])) *
                              np.sign(self.thrust_list[i])*20 *
                              (self.Main_speed / self.Main_thrust), 0)
                # 夏天星师兄的方法，比较均匀
                # speed = round(self.thrust_list[i] * (self.Main_speed / self.Main_thrust), 0)
                # 主推转速限制
                if speed < -1 * self.Main_speed:
                    speed = -1 * self.Main_speed
                elif speed > self.Main_speed:
                    speed = self.Main_speed
                self.speed_list.append(int(speed))
            else:
                # 褚老师的方法，前面力量大，后面力量小
                speed = round(math.sqrt(abs(self.thrust_list[i]))
                              * np.sign(self.thrust_list[i])*8
                              * (self.Lat_speed / self.Lat_thrust), 0)
                # 夏天星师兄的方法，比较均匀
                # speed = round(self.thrust_list[i] * (self.Lat_speed / self.Lat_thrust), 0)
                # 侧推垂推转速限制
                if speed < -1 * self.Lat_speed:
                    speed = -1 * self.Lat_speed
                elif speed > self.Lat_speed:
                    speed = self.Lat_speed
                self.speed_list.append(int(speed))
                # self.speed_list 顺序为 主推 前侧 后侧 前垂 后垂
        # print("推力分配打印：", self.speed_list)
        return self.speed_list


if __name__ == '__main__':
    # Fx: -400~400;    Fx = 主推最大推力 = 400
    # Fy: -130~130;    Fy = 侧推最大推力 * 2 = 65 *  2 = 130
    # Fz: -130~130;    Fz = 垂推最大推力 * 2 = 65 *  2 = 130
    # N : -117~117;    N  = 侧推最大推力 * 两侧推间距的一半 * 2 = 65 * 1.8/2 * 2 = 117
    # M : -130~130;    M  = 垂推最大推力 * 垂侧推间距的一半 * 2 = 65 * 2/2   * 2 = 130
    TD = ThrustDis()
    res = TD.thrust_distribute(0, 0, 0, 40, 0)
    print(res)
