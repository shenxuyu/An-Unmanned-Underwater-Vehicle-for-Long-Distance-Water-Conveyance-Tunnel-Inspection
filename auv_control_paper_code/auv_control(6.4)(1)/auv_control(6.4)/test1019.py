#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function  # 为了在Python 2中使用Python 3的print函数
import numpy as np
import casadi as ca
import rospy
from uuv_control_interfaces import DPControllerBase
from mpc_timer import UseTimer
from scipy.optimize import minimize
from mpc_test.msg import mpc_control_state
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
from sklearn.cluster import KMeans
from scipy.linalg import cholesky, inv, solve_discrete_are
from scipy.stats import norm
import time
from my_gp_class import MYGP
from trajectory import *

class ROV_MPCController(DPControllerBase):
    def __init__(self, X_train=None, Y_train=None, X_inducing=None, *args):
        DPControllerBase.__init__(self, *args)

        self.gp = None # 初始化gp属性

        """AUV 模型参数"""
        self.m = 1862.87
        self.I_Z = 691.23
        self.X_u, self.Y_v, self.N_r = 779.79, 1222, 224.32
        self.X_U, self.Y_V, self.N_R = -74.82, -69.48, -105
        self.D_U, self.D_V, self.D_R = -748.22, -992.53, -523.27
        self.M_X = self.m - self.X_u
        self.M_Y = self.m - self.Y_v
        self.M_THETA = self.I_Z - self.N_r

        """初始化控制器参数"""
        self.Nx = 3  # x,y,psi
        self.Nx_ext = 2 * self.Nx  # x,y,psi,u,v,r
        self.Nu = 3  # Fx,Fy,Fn
        self.N_predict = 10  # 预测时域
        self.Q = np.eye(self.Nx_ext) * 1e-1  # 状态惩罚矩阵
        self.Q[0, 0] = 10
        self.Q[1,1] = 10
        self.Q[2,2] = 0.1
        self.Q_end = 2 * np.eye(self.Nx_ext)  # 终端惩罚矩阵
        self.R = np.eye((self.Nu, self.Nu))  # 控制量惩罚矩阵

        '''状态和控制量初始化'''
        self.State_Initial = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00]  # yaw, r
        self.Control_init = [0.00, 0.00, 0.00] * self.N_predict  # Fr

        '''约束初始化'''
        self.state_l = [-np.inf, -np.inf, -np.inf, -5, -5, -np.pi/2]  # yaw, r
        self.state_u = [np.inf, np.inf, np.inf, 5, 5, np.pi/2]
        self.control_l = [-1000, -1000, -1000]  # Fr
        self.control_u = [1000, 1000, 1000]


        # Generate reference trajectory
        self.generate_reference_trajectory()

        # ROS setup
        self._tau = np.zeros(6)
        self.mpc_state = mpc_control_state()
        self.refg_state_sub = rospy.Subscriber('/mpc_state', mpc_control_state, self.control_state_callback)
        self.mpc_timer = UseTimer(0.1, self.update_mpc_controller)
        self.mpc_timer.timer_start()

    # ================== 生成参考轨迹=====================
    def generate_reference_trajectory(self):
        N = 1000
        t0 = np.arange(0, (N + self.N_p) * self.delta_t, self.delta_t)
        self.X_ref = 0.5 * t0
        self.Y_ref = 3 * np.sin(0.5 * t0)
        self.PHI_ref = np.arctan2(np.gradient(self.Y_ref), np.gradient(self.X_ref))
        self.U_ref = np.sqrt(np.gradient(self.X_ref)**2 + np.gradient(self.Y_ref)**2) / self.delta_t
        self.V_ref = np.zeros_like(self.U_ref)
        self.R_ref = np.gradient(self.PHI_ref) / self.delta_t
        
    def get_reference_state(self, i):
        return np.array([self.X_ref[i], self.Y_ref[i], self.PHI_ref[i], 
                         self.U_ref[i], self.V_ref[i], self.R_ref[i]])

    
    # ================= AUV状态回调 ======================
    def control_state_callback(self, msg):
        if not isinstance(msg, mpc_control_state):
            return
        self.mpc_state = msg
        self.mpc_state.phi = np.pi * (msg.phi / 180)

    # ================= MPC控制器更新 ======================
    def mpc_update(self,pre_feedbackcourse, pre_feedback_r, pre_feedback_dvl_x, pre_feedback_dvl_y):

        # *****************************************************************************************
        """gp模型初始化"""   
        Z = np.load("/home/sxy/catkin_ws/src/mpc_tes/scripts/mpc/TrainX.npy") # 加载数据 
        Y = np.load("/home/sxy/catkin_ws/src/mpc_tes/scripts/mpc/y.npy")
        Y = np.array(Y).reshape(-1, 1)  # 这里将 Y 转换为二维数组

        print(f"Z shape: {Z.shape}, Y shape: {Y.shape}") # 确保数据已正确加载

        N_hyp = len(Z) // 3 # 超参数测试数据量
    
    
        X_hyp = Z[:N_hyp]
        Y_hyp = Y[:N_hyp] # 用于GP超参数优化的训练集'
 
        X_test = Z[N_hyp:]
        Y_test = Y[N_hyp:] # 用于生成GP预测模型的训练集
    
        print(f"X_hyp shape: {X_hyp.shape}, Y_hyp shape: {Y_hyp.shape}")
        print(f"X_test shape: {X_test.shape}, Y_test shape: {Y_test.shape}") # 打印分割后的数据形状以进行调试

        gp = MYGP(X_hyp, Y_hyp, X_test, Y_test) # 初始化GP模型
        self.gp = gp
        print('=================')
        print("GP模型初始化成功")
        print('=================')

        # *****************************************************************************************
        

        self.current_time = time.time()  # 更新当前时间
        dt = self.current_time - self.last_time   # 计算采样时间
        dt = max(dt, self.sample_time)  # 防止除以零的情况

        # ***************************************************************************************
        """定义相关方程"""
        # 通过训练数据与名义模型得到矫正后模型的casadi表达式
        # ode_nominal : 名义模型
        # get_ode_ca : 将名义模型转换成casadi表达式ode_nominal_ca
        # my_gp_error_fun_ca : 通过训练的gp获得需要的均值
        # my_gp_correct_fun_ca : 将ode_nominal_ca和my_gp_error_fun_ca整合(x_dot = f(x,u) + gp(x,u))
        #
        '''获取名义模型的Casadi表达式'''
        ode_nominal_ca = self.get_ode_ca(self.ode_nominal,self.Nx, self.Nu)
        #
        '''获取通过训练的GP获得的误差模型'''
        my_gp_error_fun_ca = self.gp.get_mean_fun()
        #
        '''合并名义模型和误差模型'''
        my_gp_correct_fun_ca = self.merge_model(ode_nominal_ca, my_gp_error_fun_ca, self.Nx, self.Nu)
        #
        '''将得到的矫正后动力学模型拓展为运动学模型'''
        kine_fun_correct_pre_ca = self.dyna_2_kine_new(my_gp_correct_fun_ca, self.Nx, self.Nu)
        #
        '''将矫正后的模型进行离散化,使用RK4方法,得到离散后的casadi表达式'''
        kine_fun_correct_rk4 = self.my_rk4_fun(kine_fun_correct_pre_ca, dt, self.Nx_ext, self.Nu)
        # ******************************************************************************************


        # ******************************************************************************************
        """"初始化"""
        '''定义求解器'''
        solver = get_MPC_solver(kine_fun_correct_rk4, self.Nx_ext, self.Nu, self.N_predict, self.Q, self.Q_end, self.R)

        '''约束'''
        lbx = []  # 最低约束条件(nlp问题的求解变量，该问题中为N_predict的控制输出)
        ubx = []  # 最高约束条件
        lbg = []  # 等式最低约束条件(nlp问题的等式，该问题中为下一时刻状态与此时刻的状态与控制量)
        ubg = []  # 等式最高约束条件
        for _ in range(self.N_predict):
            lbx.append(self.control_l)
            ubx.append(self.control_u)
        for _ in range(self.N_predict + 1):
            lbg.append(self.state_l)
            ubg.append(self.state_u)
        lbg = np.array(lbg).reshape(-1, 1)
        ubg = np.array(ubg).reshape(-1, 1)
        lbx = np.array(lbx).reshape(-1, 1)
        ubx = np.array(ubx).reshape(-1, 1)

        '''初始值'''
        self.State_Initial[0] = self.mpc_state.x
        self.State_Initial[1] = self.mpc_state.y
        self.State_Initial[2] = self.mpc_state.phi
        self.State_Initial[3] = self.mpc_state.u
        self.State_Initial[4] = self.mpc_state.v
        self.State_Initial[5] = self.mpc_state.r
        print('初始状态：', self.State_Initial)
        x_init_list = self.State_Initial  # 状态初始值
        u_init_list = self.Control_init  # 输入初始值
        self.x_init = np.array(x_init_list).reshape(-1, 1)  # 每一步的状态
        self.u_init = np.array(u_init_list).reshape(-1, self.Nu)  # nlp问题求解的初值，求解器可以在此基础上优化至最优值
        '''载入参考轨迹'''
        state_ref = get_mpc_ref(x_fun, y_fun, yaw_fun, vx_fun, vy_fun, theta_fun, dt, t0, N_predict)
        # state_all_ref = self.state_ref[:, 0].reshape(-1, 1)
        # print(state_ref)


        index_t = []  # 存储时间戳，以便计算每一步求解的时间
        # 初始化优化参数
        c_p = np.concatenate((self.x_init, state_ref), axis=1).T.reshape((-1, 1))  # 增加列数量
        # 初始化优化目标变量
        init_control = ca.reshape(self.u_init, -1, 1)
        # 计算结果并且计时
        t_ = time.time()
        res = solver(x0=init_control, p=c_p, lbg=lbg, lbx=lbx, ubg=ubg, ubx=ubx)
        index_t.append(time.time() - t_)
        print('计算一次所需时间：', index_t)
        # 获得最优控制结果u
        self.u_sol = ca.reshape(res['x'], self.Nu, self.N_predict).T  # 将其恢复U的形状定义
        # gp参数的更新，每次更新1个输入，1次更新一次求解器
        u_this = self.u_sol[0, :]  # 仅将第一步控制作为真实控制输出
        self.x_sub = self.x_init[1:].T
        self.u_sub = u_this[0,:] # 提取第一行所有元素

        self._tau[0] = u_this[0]
        self._tau[1] = u_this[1]
        self._tau[2] = -259
        self._tau[5] = u_this[2]

        print("MPC输出：", u_this)
        # *****************************************************************************************
        

    def update_mpc_controller(self):
        self.mpc_update()  # Fixed: Added missing parentheses
        self.publish_control_wrench(self._tau)
        self.x0[0] = self.mpc_state.x
        self.x0[1] = self.mpc_state.y
        self.x0[2] = self.mpc_state.phi
        self.x0[3] = self.mpc_state.u
        self.x0[4] = self.mpc_state.v
        self.x0[5] = self.mpc_state.r
        self.gp_model_update()
        self.t += 1  # TODO: Consider using actual time instead of incrementing
        #if self.t >= self.N_p - 3:
        #    self.mpc_timer.timer_cancel()  # Fixed: Changed 'timer_cancle' to 'timer_cancel'

    def updata_controller(self):

        pass


    def gp_model_update(self):
        """更新GP模型"""
        # 使用新的误差方程来创建casadi函数
        ode_error_ca = self.get_ode_ca_distrub(self.ode_error_yaw, self.Nx, self.Nu)
    
        # 计算当前误差
        error_this = ode_error_ca(self.x_sub, self.u_sub)
    
        # 更新GP模型数据
        x_data = np.hstack([self.x_sub, self.u_sub])
        self.gp.data_update_new(x_data, error_this)
    
        # 更新初始状态和控制输入
        self.u_init = self.u_sol
    


    # 定义定向名义模型
    def ode_nominal(self, x, u):
        """
        名义动力学模型
        x: x[0]为x轴向速度  x[1]为y轴轴向速度   x[2]为yaw角角速度
        u: u[0]为x轴推力  u[1]为y轴推力  u[2]为z轴转动力矩
        """
        dxdt = [(self.M_Y * x[1] * x[2] + self.X_U * x[0] + self.D_U * ca.fabs(x[0]) * x[0] + u[0]) / self.M_X,
                (-self.M_X * x[0] * x[2] + self.Y_V * x[1] + self.D_V * ca.fabs(x[1]) * x[1] + u[1]) / self.M_Y,
                ((self.M_X - self.M_Y) * x[0] * x[1] + self.N_R * x[2] + self.D_R * ca.fabs(x[2]) * x[2] + u[2]) / self.M_THETA

        ]
        return dxdt
    

    # 待测参数 m_psi, D_r, N_r
    
    def ode_error_yaw(self, x, u):
        """
        误差方程
        x: x[0]为x轴向速度  x[1]为y轴轴向速度   x[2]为yaw角角速度
        u: u[0]为x轴推力  u[1]为y轴推力  u[2]为z轴转动力矩
        self.mpc_state.u: 当前x轴向速度
        self.mpc_state.v: 当前y轴轴向速度
        self.mpc_state.r: 当前yaw角角速度
        
        """
        dxdt = [ self.mpc_state.u- (self.M_Y * x[1] * x[2] + self.X_U * x[0] + self.D_U * ca.fabs(x[0]) * x[0] + u[0]) / self.M_X,
                 self.mpc_state.v- (-self.M_X * x[0] * x[2] + self.Y_V * x[1] + self.D_V * ca.fabs(x[1]) * x[1] + u[1]) / self.M_Y,
                 self.mpc_state.r- ((self.M_X - self.M_Y) * x[0] * x[1] + self.N_R * x[2] + self.D_R * ca.fabs(x[2]) * x[2] + u[2]) / self.M_THETA
                ]
        return dxdt



    # 定义定向修正模型
    def merge_model(self, nominal_fun, error_fun, Nx, Nu):
        """将名义模型与误差模型混合"""
        X = ca.SX.sym('x', Nx)
        if Nu != 0:
            U = ca.SX.sym('u', Nu)
            # 联合构建修正后的模型
            correct_fun = ca.Function("correct_fun", [X, U], [nominal_fun(X, U) + error_fun(X, U)])
        else:
            correct_fun = ca.Function("correct_fun", [X], [nominal_fun(X) + error_fun(X)])
        return correct_fun
    



    def get_ode_ca_distrub(self, ode, Nx, Nu):
        """将list形式的ode表达式构建成casadi表达式"""
        X = ca.SX.sym('x', Nx)
        U = ca.SX.sym('u', Nu)
        ode_expr = ode(X, U)
        ode_ca = ca.Function("ode_ca", [X, U], [ca.vertcat(*ode_expr)])
        return ode_ca
    

    def get_ode_ca(self, ode, Nx, Nu):
        """
        将list形式的ode表达式构建成casadi表达式
        """
        X = ca.SX.sym('x',Nx)
        U = ca.SX.sym('u',Nu)
        ode_ca = ca.Function("ode_ca", [X, U], [ca.vertcat(*ode(X, U))])
        return ode_ca




 
    def dyna_2_kine(self,ode_dyna, Nx, Nu):
        """将动力学方程扩展为运动学方程，目前的方法是将状态量扩展为位置+速度"""
        x_extern = ca.SX.sym('x_extern', 2 * Nx)
        dx_extern = ca.SX.sym('dx_extern', 2 * Nx)
        U = ca.SX.sym('u', Nu)
        dx_extern[:Nx] = x_extern[Nx:]
        dx_extern[Nx:] = ode_dyna(x_extern[Nx:], U)
        Kine_fun_ca = ca.Function("kine_fun_ca", [x_extern, U], [dx_extern])
        return Kine_fun_ca
    

    def dyna_2_kine_new(ode_dyna, Nx, Nu):
        """此函数的目的是为了将动力学方程扩展为运动学方程，目前的方法是将状态量扩展为位置+速度，此函数目前仅仅为水平面三自由度运动提供转换
        已完成
        """
        x_extern = ca.SX.sym('x_extern', 2 * Nx)
        dx_extern = ca.SX.sym('dx_extern', 2 * Nx)
        U = ca.SX.sym('u',Nu)
        dx_extern[0] = x_extern[3] * ca.cos(x_extern[2]) - x_extern[4] * ca.sin(x_extern[2])
        dx_extern[1] = x_extern[3] * ca.sin(x_extern[2]) + x_extern[4] * ca.cos(x_extern[2])
        dx_extern[2] = x_extern[5]
        dx_extern[3:] = ode_dyna(x_extern[3:], U)
        Kine_fun_ca = ca.Function("kine_fun_ca", [x_extern, U], [dx_extern])
        # print(Kine_fun_ca)
        return Kine_fun_ca
    

    def my_rk4_fun(self, ode, dt, Nx, Nu):
        """创建离散RK4模型"""
        X = ca.SX.sym('x', Nx)
        U = ca.SX.sym('u', Nu)
        ode_casadi = ca.Function("ode", [X, U], [ode(X, U)])
        k1 = ode_casadi(X, U)
        k2 = ode_casadi(X + dt / 2 * k1, U)
        k3 = ode_casadi(X + dt / 2 * k2, U)
        k4 = ode_casadi(X + dt * k3, U)
        xrk4 = X + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        rk4 = ca.Function("ode_rk4", [X, U], [xrk4])
        return rk4

   

  
