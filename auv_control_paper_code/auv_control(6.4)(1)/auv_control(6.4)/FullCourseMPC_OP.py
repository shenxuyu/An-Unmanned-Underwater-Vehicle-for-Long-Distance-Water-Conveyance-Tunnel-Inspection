#!/usr/bin/env python3
# coding: utf-8
import time
import casadi as ca
import numpy as np
from scipy.stats import norm
import casadi.tools as ctools

# 导入自定义模块
from my_gp_class import MYGP
from MPC_solver import get_MPC_solver
import scipy.linalg

class FullCourseMPC:
   def __init__(self, sample_time=0.00):
       self.sample_time = sample_time  # 采样时间
       self.current_time = time.time()  # 当前时间
       self.current_state = 0.0  # 当前状态
       self.last_time = self.current_time  # 上一次更新时间
       self.gp = None  # 初始化 GP 属性
       self.dt = 0.05  # 采样时间间隔
       self.horizon = 5  # 预测时域
       Nt = int(self.horizon / self.dt)

       # 初始化控制器参数
       self.Nx = 1  # 状态量个数：航向(yaw)
       self.Nx_ext = 2 * self.Nx  # 扩展状态：航向(yaw)、绕 z 轴的角速度(r)
       self.Nu = 1  # 控制量个数
       self.N_predict = 5  # 预测时域
       self.Q = np.eye(self.Nx_ext) * 1e-1  # 状态惩罚矩阵
       self.Q[0, 0] = 10
       self.Q[1, 1] = 0.1
       self.Q_end = 2 * np.eye(self.Nx_ext)  # 终端惩罚矩阵
       self.R = np.diag([1e-8])  # 控制量惩罚矩阵

       # 约束初始化
       self.state_l = [-np.inf, -np.pi / 2]  # yaw, r
       self.state_u = [np.inf, np.pi / 2]
       self.control_l = [-117]  # Fr
       self.control_u = [117]

       # 状态和控制量初始化
       self.State_Initial = [0, 0]  # yaw, r
       self.Control_init = [0] * self.N_predict  # Fr
       self.output = 0.00  # 力输出(Fr)

       # 参考值初始化
       self.state_ref = np.zeros((self.Nx_ext, self.N_predict + 1))  # x 参考值维度
       self.clear()

       # 常数和反馈信号初始化
       self.m_psi = -0.341607
       self.D_r = 13.955754
       self.N_r = 0.149469
       self.N_r_abs = 0.021017

       # 定义参考信号的平均值
       mean_ref_s = ca.MX.sym('mean_ref', self.Nx)
       # 定义终端状态矩阵
       P_s = ca.MX.sym('P', self.Nx * self.Nx)
       # 定义增益矩阵
       K_s            = ca.MX.sym('K', self.Nu * self.Nx)

       """ 默认百分位概率 """
       # 定义置信度因子，这里取95%置信度因子
       percentile = 0.95
       # 使用正态分布的分位数函数计算quantile_x和quantile_u
       quantile_x = np.ones(self.Nx) * norm.ppf(percentile)
       quantile_u = np.ones(self.Nu) * norm.ppf(percentile)
       # 初始化Hx和Hu
       Hx = ca.MX.eye(self.Nx)
       Hu = ca.MX.eye(self.Nu)


       """ 定义要使用的成本函数 """
       self.__set_cost_function(mean_ref_s, P_s.reshape((self.Nx, self.Nx)))


       """ Feedback function """
       mean_s = ca.MX.sym('mean', self.Nx)
       v_s = ca.MX.sym('v', self.Nu)   
       u_func = ca.Function('u', [mean_s, mean_ref_s, v_s, K_s],
                                 [v_s + ca.mtimes(K_s.reshape((self.Nu, self.Nx)),
                                 mean_s-mean_ref_s)])

       """ 创建变量结构 """
       var = ctools.struct_symMX([(
               ctools.entry('mean', shape=(self.Nx,), repeat=Nt + 1),
               ctools.entry('L', shape=(int((self.Nx**2 - self.Nx)/2 + self.Nx),), repeat=Nt + 1),
               ctools.entry('v', shape=(Nu,), repeat=Nt),
               ctools.entry('eps', shape=(3,), repeat=Nt + 1),
               ctools.entry('eps_state', shape=(self.Nx,), repeat=Nt + 1),
            )])
       

       num_slack = 3 
       num_state_slack = Ny
       self.__var = var
       self.__num_var = var.size

       # 决策变量边界
       self.__varlb = var(-np.inf)
       self.__varub = var(np.inf)

       """ 调整硬边界 """
       for t in range(Nt + 1):
            j = self.Nx
            k = 0
            for i in range(self.Nx):
                # Lower boundry of diagonal
                self.__varlb['L', t, k] = 0
                k += j
                j -= 1
            self.__varlb['eps', t] = 0
            self.__varlb['eps_state', t] = 0
            if xub is not None:
                self.__varub['mean', t] = xub
            if xlb is not None:
                self.__varlb['mean', t] = xlb
            if lam_state is None:
                self.__varub['eps_state'] = 0


       """ 输入协方差矩阵 """

       N_gp, Ny_gp, Nu_gp = self.__gp.get_size()
       Nz_gp = Ny_gp + Nu_gp
       covar_d_sx = ca.SX.sym('cov_d', Ny_gp, Ny_gp)
       K_sx = ca.SX.sym('K', Nu, Ny)
       covar_u_func = ca.Function('cov_u', [covar_d_sx, K_sx], [ca.SX(Nu, Nu)])
       covar_s = ca.SX(Nz_gp, Nz_gp)
       covar_s[:Ny_gp, :Ny_gp] = covar_d_sx
       covar_func = ca.Function('covar', [covar_d_sx], [covar_s])
       

       """ 混合输出协方差矩阵 """
   
       N_gp, Ny_gp, Nu_gp = self.__gp.get_size()
       covar_d_sx = ca.SX.sym('covar_d', Ny_gp, Ny_gp)
       covar_x_sx = ca.SX.sym('covar_x', Ny, Ny)
       u_s       = ca.SX.sym('u', Nu)

       cov_x_next_s = ca.SX(Ny, Ny)
       cov_x_next_s[:Ny_gp, :Ny_gp] = covar_d_sx
       covar_x_next_func = ca.Function( 'cov', [covar_d_sx], [cov_x_next_s])


       L_s = ca.SX.sym('L', ca.Sparsity.lower(Ny))
       L_to_cov_func = ca.Function('cov', [L_s], [L_s @ L_s.T])
       covar_x_sx = ca.SX.sym('cov_x', Ny, Ny)
       cholesky = ca.Function('cholesky', [covar_x_sx], [ca.chol(covar_x_sx).T])

       """ 设置初始值 """
       obj = ca.MX(0)
       con_eq = []
       con_ineq = []
       con_ineq_lb = []
       con_ineq_ub = []
       con_eq.append(var['mean', 0] - mean_0_s)
       L_0_s = ca.MX(ca.Sparsity.lower(Ny), var['L', 0])
       L_init = cholesky(covariance_0_s.reshape((Ny,Ny)))
       con_eq.append(L_0_s.nz[:]- L_init.nz[:])
       u_past = u_0_s


       """ 建立约束 """
       for t in range(Nt):
            # Input to GP
            mean_t = var['mean', t]
            u_t = u_func(mean_t, mean_ref_s, var['v', t], K_s)
            L_x = ca.MX(ca.Sparsity.lower(Ny), var['L', t])
            covar_x_t = L_to_cov_func(L_x)

            N_gp, Ny_gp, Nu_gp = self.__gp.get_size()
            covar_t = covar_func(covar_x_t[:Ny_gp, :Ny_gp])

            """ Select the chosen integrator """
   
            N_gp, Ny_gp, Nu_gp = self.__gp.get_size()
            mean_d, covar_d = self.__gp.predict(mean_t[:Ny_gp], u_t, covar_t)
            mean_next_pred = ca.vertcat(mean_d, hybrid.rk4(mean_t[Ny_gp:], mean_t[:Ny_gp], []))
            covar_x_next_pred = covar_x_next_func(covar_d )
    
            """ 连续约束 """
            mean_next = var['mean', t + 1]
            con_eq.append(mean_next_pred - mean_next )

            L_x_next = ca.MX(ca.Sparsity.lower(Ny), var['L', t + 1])
            covar_x_next = L_to_cov_func(L_x_next).reshape((Ny*Ny,1))
            L_x_next_pred = cholesky(covar_x_next_pred)
            con_eq.append(L_x_next_pred.nz[:] - L_x_next.nz[:])

            """ 机会状态约束 """
            cons = self.__constraint(mean_next, L_x_next, Hx, quantile_x, xub,
                                    xlb, var['eps_state',t])
            con_ineq.extend(cons['con'])
            con_ineq_lb.extend(cons['con_lb'])
            con_ineq_ub.extend(cons['con_ub'])

            """ 输入约束 """
            cov_u = ca.MX(Nu, Nu)

            if uub is not None:
                con_ineq.append(u_t)
                con_ineq_ub.extend(uub)
                con_ineq_lb.append(np.full((Nu,), -ca.inf))
            if ulb is not None:
                con_ineq.append(u_t)
                con_ineq_ub.append(np.full((Nu,), ca.inf))
                con_ineq_lb.append(ulb)

            """ 添加额外约束 """
            if inequality_constraints is not None:
                cons = inequality_constraints(var['mean', t + 1],
                                              covar_x_next,
                                              u_t, var['eps', t], con_par)
                con_ineq.extend(cons['con_ineq'])
                con_ineq_lb.extend(cons['con_ineq_lb'])
                con_ineq_ub.extend(cons['con_ineq_ub'])

            """ 目标函数 """
            u_delta = u_t - u_past
            obj += self.__l_func(var['mean', t], covar_x_t, u_t, cov_u, u_delta) \
                    + np.full((1, num_slack),lam) @ var['eps', t]
            if lam_state is not None:
                obj += np.full((1,num_state_slack),lam_state) @ var['eps_state', t]
            u_t = u_past
       L_x = ca.MX(ca.Sparsity.lower(Ny), var['L', Nt])
       covar_x_t = L_to_cov_func(L_x)
       obj += self.__lf_func(var['mean', Nt], covar_x_t, P_s.reshape((Ny, Ny))) \
            + np.full((1, num_slack),lam) @ var['eps', Nt]
       if lam_state is not None:
            obj += np.full((1,num_state_slack),lam_state) @ var['eps_state', Nt]


       num_eq_con = ca.vertcat(*con_eq).size1()
       num_ineq_con = ca.vertcat(*con_ineq).size1()
       con_eq_lb = np.zeros((num_eq_con,))
       con_eq_ub = np.zeros((num_eq_con,))

       """ 终端约束 """
  
       con_ineq.append(self.__lf_func(var['mean', Nt],
                            covar_x_t, P_s.reshape((Ny, Ny))))
       num_ineq_con += 1
       con_ineq_lb.append(0)
       con_ineq_ub.append(terminal_constraint)
       con = ca.vertcat(*con_eq, *con_ineq)
       self.__conlb = ca.vertcat(con_eq_lb, *con_ineq_lb)
       self.__conub = ca.vertcat(con_eq_ub, *con_ineq_ub)

       """ 建立求解器 """
       nlp = dict(x=var, f=obj, g=con, p=param_s)
       options = {
            'ipopt.print_level' : 0,
            'ipopt.mu_init' : 0.01,
            'ipopt.tol' : 1e-8,
            'ipopt.warm_start_init_point' : 'yes',
            'ipopt.warm_start_bound_push' : 1e-9,
            'ipopt.warm_start_bound_frac' : 1e-9,
            'ipopt.warm_start_slack_bound_frac' : 1e-9,
            'ipopt.warm_start_slack_bound_push' : 1e-9,
            'ipopt.warm_start_mult_bound_push' : 1e-9,
            'ipopt.mu_strategy' : 'adaptive',
            'print_time' : False,
            'verbose' : False,
            'expand' : True
            }

       options.update(solver_opts)
       self.__solver = ca.nlpsol('mpc_solver', 'ipopt', nlp, options)

        # First prediction used in the NLP, used in plot later
       self.__var_prediction = np.zeros((Nt + 1, Ny))
       self.__mean_prediction = np.zeros((Nt + 1, Ny))
       self.__mean = None

       build_solver_time += time.time()
       print('\n________________________________________')
       print('# Time to build mpc solver: %f sec' % build_solver_time)
       print('# Number of variables: %d' % self.__num_var)
       print('# Number of equality constraints: %d' % num_eq_con)
       print('# Number of inequality constraints: %d' % num_ineq_con)
       print('----------------------------------------')


   def __set_cost_function(self, mean_ref_s, P_s):
        """ 
        定义阶段成本和终点成本
        costFunc: 目标中使用的成本函数，二次成本的期望值
        """
        mean_s = ca.MX.sym('mean', self.__Ny)
        covar_x_s = ca.MX.sym('covar_x', self.__Ny, self.__Ny)
        covar_u_s = ca.MX.sym('covar_u', self.__Nu, self.__Nu)
        u_s = ca.MX.sym('u', self.__Nu)
        delta_u_s = ca.MX.sym('delta_u', self.__Nu)
        Q = ca.MX(self.__Q)
        R = ca.MX(self.__R)
        S = ca.MX(self.__S)    
        self.__l_func = ca.Function('l', [mean_s, covar_x_s, u_s,
                                                covar_u_s, delta_u_s],
                               [self.__cost_l(mean_s, mean_ref_s, covar_x_s, u_s,
                                covar_u_s, delta_u_s, Q, R, S)])
        self.__lf_func = ca.Function('lf', [mean_s, covar_x_s, P_s],
                                   [self.__cost_lf(mean_s, mean_ref_s, covar_x_s, P_s)])
        

   def __cost_lf(self, x, x_ref, covar_x, P, s=1):
        """ 终端成本函数： 二次成本的预期值
        """
        P_s = ca.SX.sym('Q', ca.MX.size(P))
        x_s = ca.SX.sym('x', ca.MX.size(x))
        covar_x_s = ca.SX.sym('covar_x', ca.MX.size(covar_x))

        sqnorm_x = ca.Function('sqnorm_x', [x_s, P_s],
                               [ca.mtimes(x_s.T, ca.mtimes(P_s, x_s))])
        trace_x = ca.Function('trace_x', [P_s, covar_x_s],
                               [s * ca.trace(ca.mtimes(P_s, covar_x_s))])
        return sqnorm_x(x - x_ref, P) + trace_x(P, covar_x)
   

   def __cost_l(self, x, x_ref, covar_x, u, covar_u, delta_u, Q, R, S, s=1):
        """ 阶段成本函数： 二次成本的期望值
        """
        Q_s = ca.SX.sym('Q', ca.MX.size(Q))
        R_s = ca.SX.sym('R', ca.MX.size(R))
        x_s = ca.SX.sym('x', ca.MX.size(x))
        u_s = ca.SX.sym('u', ca.MX.size(u))
        covar_x_s = ca.SX.sym('covar_x', ca.MX.size(covar_x))
        covar_u_s = ca.SX.sym('covar_u', ca.MX.size(R))

        sqnorm_x = ca.Function('sqnorm_x', [x_s, Q_s],
                               [ca.mtimes(x_s.T, ca.mtimes(Q_s, x_s))])
        sqnorm_u = ca.Function('sqnorm_u', [u_s, R_s],
                               [ca.mtimes(u_s.T, ca.mtimes(R_s, u_s))])
        trace_u  = ca.Function('trace_u', [R_s, covar_u_s],
                               [s * ca.trace(ca.mtimes(R_s, covar_u_s))])
        trace_x  = ca.Function('trace_x', [Q_s, covar_x_s],
                               [s * ca.trace(ca.mtimes(Q_s, covar_x_s))])

        return sqnorm_x(x - x_ref, Q) + sqnorm_u(u, R) + sqnorm_u(delta_u, S) \
                + trace_x(Q, covar_x)  + trace_u(R, covar_u)
   

   def __constraint(self, mean, covar, H, quantile, ub, lb, eps):
        """ 建立机会约束向量
        """

        r = ca.SX.sym('r')
        mean_s = ca.SX.sym('mean', ca.MX.size(mean))
        S_s = ca.SX.sym('S', ca.MX.size(covar))
        H_s = ca.SX.sym('H', 1, ca.MX.size2(H))
        S = covar
        con_func = ca.Function('con', [mean_s, S_s, H_s, r],
                                [H_s @ mean_s + r * H_s @ ca.diag(S_s)])

        con = []
        con_lb = []
        con_ub = []
        for i in range(ca.MX.size1(mean)):
            con.append(con_func(mean, S, H[i, :], quantile[i]) - eps[i])
            con_ub.append(ub[i])
            con_lb.append(-np.inf)
            con.append(con_func(mean, S, H[i, :], -quantile[i]) + eps[i])
            con_ub.append(np.inf)
            con_lb.append(lb[i])
        cons = dict(con=con, con_lb=con_lb, con_ub=con_ub)
        return cons
   


   def solve(self, x0, sim_time, x_sp=None, u0=None, debug=False, noise=False,
              con_par_func=None):
        """ 求解优化控制问题

        # Arguments:
            x0: 初始状态向量

        # Optional Arguments:
            x_sp: 状态设置点，默认为零。
            u0: 初始输入向量
            debug: 如果为 True，则在每次求解迭代时打印调试信息。
            noise: 如果为 True，则在模拟中添加高斯噪声。
            con_par_func: 函数，用于计算传递给不等式函数的参数，输入当前状态。

        # Returns:
            mean: 使用最佳控制输入模拟输出
            u: 优化控制输出
        """

        Nt = self.__Nt
        Ny = self.__Ny
        Nu = self.__Nu
        dt = self.__dt

        # 初始化状态
        if u0 is None:
            u0 = np.zeros(Nu)
        if x_sp is None:
            self.__x_sp = np.zeros(Ny)
        else:
            self.__x_sp = x_sp

        self.__Nsim = int(sim_time / dt)

        # 初始化变量
        self.__mean          = np.full((self.__Nsim + 1, Ny), x0)
        self.__mean_pred     = np.full((self.__Nsim + 1, Ny), x0)
        self.__covariance    = np.full((self.__Nsim + 1, Ny, Ny), np.eye(Ny) * 1e-8)
        self.__u             = np.full((self.__Nsim, Nu), u0)

        self.__mean[0]       = x0
        self.__mean_pred[0]  = x0
        self.__covariance[0] = np.eye(Ny)*1e-10 #np.diag(self.__variance_0)
        self.__u[0]          = u0

        # 热启动变量的初步猜测
        self.__var_init = self.__var(0)

        # 增加 cov cholesky 的初始化
        cov0 = self.__covariance[0]
        self.__var_init['L', 0] = cov0[np.tril_indices(Ny)]
        self.__lam_x0 = np.zeros(self.__num_var)
        self.__lam_g0 = 0

        """ 围绕工作点进行线性化，计算 LQR 增益矩阵 """

        N_gp, Ny_gp, Nu_gp = self.__gp.get_size()
        A_f, B_f = self.__hybrid.discrete_rk4_linearize(x0[Ny_gp:], x0[:Ny_gp])
        A_gp, B_gp = self.__gp.discrete_linearize(x0[:Ny_gp], u0, np.eye(Ny_gp+Nu_gp)*1e-8)
        A = np.zeros((Ny, Ny))
        B = np.zeros((Ny, Nu))
        A[:Ny_gp, :Ny_gp] = A_gp
        B[:Ny_gp, :] = B_gp
        K, P, E = self.lqr(A, B, self.__Q, self.__R)
 

        print('\nSolving MPC with %d step horizon' % Nt)
        for t in range(self.__Nsim):

            """ 根据测量结果更新初始值 """
            self.__var_init['mean', 0]  = self.__mean[t]

            # 获取约束参数
            con_par = con_par_func(self.__mean[t, :])

            param  = ca.vertcat(self.__mean[t, :], self.__x_sp,
                                cov0.flatten(), u0, K.flatten(),
                                P.flatten(), con_par)
            args = dict(x0=self.__var_init,
                        lbx=self.__varlb,
                        ubx=self.__varub,
                        lbg=self.__conlb,
                        ubg=self.__conub,
                        lam_x0=self.__lam_x0,
                        lam_g0=self.__lam_g0,
                        p=param)

            """ Solve nlp"""
            sol             = self.__solver(**args)
            status          = self.__solver.stats()['return_status']
            optvar          = self.__var(sol['x'])
            self.__var_init = optvar
            self.__lam_x0   = sol['lam_x']
            self.__lam_g0   = sol['lam_g']

            """ Print status """
            solve_time     += time.time()
            print("* t=%f: %s - %f sec" % (t * self.__dt, status, solve_time))

        
            for i in range(Nt + 1):
                Li = ca.DM(ca.Sparsity.lower(self.__Ny), optvar['L', i])
                cov = Li @ Li.T
                self.__var_prediction[i, :] = np.array(ca.diag(cov)).flatten()
                self.__mean_prediction[i, :] = np.array(optvar['mean', i]).flatten()

            v = optvar['v', 0, :]

            self.__u[t, :] = np.array(self.__u_func(self.__mean[t, :], self.__x_sp,
                                v, K.flatten())).flatten()
            self.__mean_pred[t + 1] = np.array(optvar['mean', 1]).flatten()
            L = ca.DM(ca.Sparsity.lower(self.__Ny), optvar['L', 1])
            self.__covariance[t + 1] = L @ L.T

            """ Simulate the next step """
            self.__mean[t + 1] = self.__model.sim(self.__mean[t],
                                       self.__u[t].reshape((1, Nu)), noise=noise)
            """下一次迭代的初始值"""
            x0 = self.__mean[t + 1]
            u0 = self.__u[t]
        return self.__mean, self.__u


   def clear(self):
       """清空状态"""
       self.exp_course = 0.0  # 目标位置
       self.output = 0.0

   def initialize_gp_model(self, data_path):
       """
       初始化 GP 模型
       
       Args:
           data_path (str): 训练数据所在路径
       """
       r, Fr, Y = load_data(data_path)
       Z = np.hstack([r, Fr])  # z = [x, u]
       print(f"Z shape: {Z.shape}, Y shape: {Y.shape}")  # 确保数据已正确加载

       N_hyp = len(Z) // 3
       print(f"N_hyp: {N_hyp}")  # 打印数据形状以进行调试

       # 用于 GP 超参数优化的训练集
       X_hyp = Z[:N_hyp]
       Y_hyp = Y[:N_hyp]

       # 用于生成 GP 预测模型的训练集
       X_test = Z[N_hyp:]
       Y_test = Y[N_hyp:]

       # 打印分割后的数据形状以进行调试
       print(f"X_hyp shape: {X_hyp.shape}, Y_hyp shape: {Y_hyp.shape}")
       print(f"X_test shape: {X_test.shape}, Y_test shape: {Y_test.shape}")

       # 初始化 GP 模型
       self.gp = MYGP(X_hyp, Y_hyp, X_test, Y_test)
       print('=================')
       print("GP 模型初始化成功")
       print('=================')

   def get_corrected_model(self):
       """
       获取矫正后的动力学模型
       
       Returns:
           kine_fun_correct_rk4 (casadi.Function): 矫正后的离散化动力学模型
       """
       # 获取名义模型的 Casadi 表达式
       ode_nominal_ca = self.get_ode_ca(self.ode_nominal_yaw, self.Nx, self.Nu)

       # 获取通过训练的 GP 获得的误差模型
       my_gp_error_fun_ca = self.gp.get_mean_fun()

       # 合并名义模型和误差模型
       my_gp_correct_fun_ca = self.merge_model(ode_nominal_ca, my_gp_error_fun_ca, self.Nx, self.Nu)

       # 将得到的矫正后动力学模型扩展为运动学模型
       kine_fun_correct_pre_ca = self.dyna_2_kine(my_gp_correct_fun_ca, self.Nx, self.Nu)

       # 将矫正后的模型进行离散化,使用 RK4 方法,得到离散后的 Casadi 表达式
       kine_fun_correct_rk4 = self.my_rk4_fun(kine_fun_correct_pre_ca, self.sample_time, self.Nx_ext, self.Nu)

       return kine_fun_correct_rk4

   def mpc_update(self, pre_feedbackcourse, pre_feedback_r, pre_feedback_dvl_x, pre_feedback_dvl_y):
       """更新 MPC 控制器状态"""
       # 获取矫正后的离散化动力学模型
       kine_fun_correct_rk4 = self.get_corrected_model()

       # 定义求解器
       solver = get_MPC_solver(kine_fun_correct_rk4, self.Nx_ext, self.Nu, self.N_predict, self.Q, self.Q_end, self.R)

       # 约束初始化
       lbx, ubx = self.get_control_constraints(self.N_predict)
       lbg, ubg = self.get_state_constraints(self.N_predict)

       # 初始值
       self.State_Initial[0] = pre_feedbackcourse
       self.State_Initial[1] = pre_feedback_r
       print('初始状态：', self.State_Initial)
       x_init_list = self.State_Initial  # 状态初始值
       u_init_list = self.Control_init  # 输入初始值
       self.x_init = np.array(x_init_list).reshape(-1, 1)  # 每一步的状态
       self.u_init = np.array(u_init_list).reshape(-1, self.Nu)  # nlp 问题求解的初值

       # 载入参考轨迹
       self.state_ref = self.get_mpc_yaw_ref(self.N_predict)

       # 初始化优化参数
       c_p = np.concatenate((self.x_init, self.state_ref), axis=1).T.reshape((-1, 1))  # 增加列数量

        # 初始化优化目标变量
       init_control = ca.reshape(self.u_init, -1, 1)

        # 计算结果并且计时
       index_t = []  # 存储时间戳,以便计算每一步求解的时间
       t_ = time.time()
       res = solver(x0=init_control, p=c_p, lbg=lbg, lbx=lbx, ubg=ubg, ubx=ubx)
       index_t.append(time.time() - t_)
       print('计算一次所需时间：', index_t)

       # 获得最优控制结果 u
       self.u_sol = ca.reshape(res['x'], self.Nu, self.N_predict).T  # 将其恢复 U 的形状定义

       # 提取第一步控制作为真实控制输出
       u_this = self.u_sol[0, :]
       self.x_sub = self.x_init[1:].T
       self.u_sub = u_this[0, :]

       # 根据控制输出限制输出值
       if u_this >= 117:
           self.output = 10
       else:
           self.output = u_this

       print("MPC 输出：", self.output)

   def gp_model_update(self, feedback_r, pre_feedback_dvl_x, pre_feedback_dvl_y):
        """更新 GP 模型"""
        # 使用新的误差方程来创建 Casadi 函数
        ode_error_ca = self.get_ode_ca_distrub(self.ode_error_yaw, feedback_r, pre_feedback_dvl_x, pre_feedback_dvl_y, self.Nx, self.Nu)

        # 计算当前误差
        error_this = ode_error_ca(self.x_sub, self.u_sub)

        # 更新 GP 模型数据
        x_data = np.hstack([self.x_sub, self.u_sub])
        self.gp.data_update_new(x_data, error_this)

        # 更新初始状态和控制输入
        self.u_init = self.u_sol

        # 更新当前时间
        self.current_time = time.time()

   def set_course(self, exp_course, feedbackcourse):
        """
        设置期望航向
        
        Args:
            exp_course (float): 期望航向
            feedbackcourse (float): 反馈航向
        """
        self.exp_course = exp_course
        self.exp_r = self.calculate_exp_r(exp_course, feedbackcourse)

   def calculate_exp_r(self, exp_course, feedbackcourse):
        """
        根据航向参考值计算角速度参考值
        
        Args:
            exp_course (float): 期望航向
            feedbackcourse (float): 反馈航向
            
        Returns:
            float: 角速度参考值
        """
        return (exp_course - feedbackcourse) / self.sample_time

   def get_mpc_yaw_ref(self, N_predict):
        """
        得到参考轨迹
        
        Args:
            N_predict (int): 预测时域长度
            
        Returns:
            numpy.ndarray: 参考轨迹,形状为 (2, N_predict + 1)
        """
        state_ref = np.empty((2, N_predict + 1))
        for i in range(N_predict + 1):
            state_ref[0, i] = self.exp_course  # 航向
            state_ref[1, i] = self.exp_r  # 角速度
        return state_ref

   def ode_nominal_yaw(self, x, u, pre_feedback_dvl_x, pre_feedback_dvl_y):
        """
        名义动力学模型
        
        Args:
            x (casadi.SX): 状态量,yaw 角角速度(r)
            u (casadi.SX): 控制量,z 轴转动力矩
            
        Returns:
            list: 状态量导数
        """
        dxdt = [
            u / self.m_psi - self.D_r * pre_feedback_dvl_x * pre_feedback_dvl_y + self.N_r * x - self.N_r_abs * x * ca.fabs(x)
        ]
        return ca.vertcat(*dxdt)
   
   def ode_hybrid(x, u, z, p):
        """ 与混合动力模型一起使用的运动方程
        """
        dxdt = [
                    u
                ]
        return  ca.vertcat(*dxdt)



   
   def get_ode_ca(self, ode, pre_feedback_dvl_x, pre_feedback_dvl_y, Nx, Nu):
        """
        将 list 形式的 ode 表达式构建成 Casadi 表达式
        
        Args:
            ode (callable): 动力学模型的函数
            Nx (int): 状态量个数
            Nu (int): 控制量个数
            
        Returns:
            casadi.Function: 动力学模型的 Casadi 函数
        """
        X = ca.SX.sym('x', Nx)
        U = ca.SX.sym('u', Nu)
        ode_ca = ca.Function("ode_ca", [X, U], [ca.vertcat(*ode(X, U, pre_feedback_dvl_x, pre_feedback_dvl_y))])
        return ode_ca

   def ode_error_yaw(self, x, u, feedback_r, pre_feedback_dvl_x, pre_feedback_dvl_y):
        """
        误差方程
        
        Args:
            x (casadi.SX): 状态量,yaw 角角速度(r)
            u (casadi.SX): 控制量,z 轴转动力矩
            feedback_r (float): 反馈角速度
            pre_feedback_dvl_x (float): 上一时刻反馈的 DVL x 方向速度
            pre_feedback_dvl_y (float): 上一时刻反馈的 DVL y 方向速度
            
        Returns:
            casadi.SX: 误差方程
        """
        dxdt = feedback_r - (u / self.m_psi - self.D_r * pre_feedback_dvl_x * pre_feedback_dvl_y + self.N_r * x - self.N_r_abs * ca.fabs(x) * x)
        return dxdt

   def merge_model(self, nominal_fun, error_fun, Nx, Nu):
        """
        将名义模型与误差模型混合
        
        Args:
            nominal_fun (casadi.Function): 名义模型的 Casadi 函数
            error_fun (casadi.Function): 误差模型的 Casadi 函数
            Nx (int): 状态量个数
            Nu (int): 控制量个数
            
        Returns:
            casadi.Function: 矫正后的模型
        """
        X = ca.SX.sym('x', Nx)
        if Nu != 0:
            U = ca.SX.sym('u', Nu)
            # 联合构建修正后的模型
            correct_fun = ca.Function("correct_fun", [X, U], [nominal_fun(X, U) + error_fun(X, U)])
        else:
            correct_fun = ca.Function("correct_fun", [X], [nominal_fun(X) + error_fun(X)])
        return correct_fun

   def get_ode_ca_distrub(self, ode, feedback_r, pre_feedback_dvl_x, pre_feedback_dvl_y, Nx, Nu):
        """
        将 list 形式的 ode 表达式构建成 Casadi 表达式
        
        Args:
            ode (callable): 误差模型的函数
            feedback_r (float): 反馈角速度
            pre_feedback_dvl_x (float): 上一时刻反馈的 DVL x 方向速度
            pre_feedback_dvl_y (float): 上一时刻反馈的 DVL y 方向速度
            Nx (int): 状态量个数
            Nu (int): 控制量个数
            
        Returns:
            casadi.Function: 误差模型的 Casadi 函数
        """
        X = ca.SX.sym('x', Nx)
        U = ca.SX.sym('u', Nu)
        ode_ca = ca.Function("ode_ca", [X, U], [ca.vertcat(*ode(X, U, feedback_r, pre_feedback_dvl_x, pre_feedback_dvl_y))])
        return ode_ca



   def dyna_2_kine(self, ode_dyna, Nx, Nu):
        """
        将动力学方程扩展为运动学方程,目前的方法是将状态量扩展为位置+速度
        
        Args:
            ode_dyna (casadi.Function): 动力学模型的 Casadi 函数
            Nx (int): 状态量个数
            Nu (int): 控制量个数
            
        Returns:
            casadi.Function: 扩展后的运动学模型
        """
        x_extern = ca.SX.sym('x_extern', 2 * Nx)
        dx_extern = ca.SX.sym('dx_extern', 2 * Nx)
        U = ca.SX.sym('u', Nu)
        dx_extern[:Nx] = x_extern[Nx:]
        dx_extern[Nx:] = ode_dyna(x_extern[Nx:], U)
        Kine_fun_ca = ca.Function("kine_fun_ca", [x_extern, U], [dx_extern])
        return Kine_fun_ca

   def my_rk4_fun(self, ode, dt, Nx, Nu):
        """
        创建离散 RK4 模型
        
        Args:
            ode (casadi.Function): 动力学模型的 Casadi 函数
            dt (float): 采样时间
            Nx (int): 状态量个数
            Nu (int): 控制量个数
            
        Returns:
            casadi.Function: 离散化后的 RK4 模型
        """
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

   def get_control_constraints(self, N_predict):
        """
        获取控制量约束
        
        Args:
            N_predict (int): 预测时域长度
            
        Returns:
            tuple: 最低约束条件和最高约束条件
        """
        lbx = []  # 最低约束条件
        ubx = []  # 最高约束条件
        for _ in range(N_predict):
            lbx.append(self.control_l)
            ubx.append(self.control_u)
        lbx = np.array(lbx).reshape(-1, 1)
        ubx = np.array(ubx).reshape(-1, 1)
        return lbx, ubx

   def get_state_constraints(self, N_predict):
        """
        获取状态量约束
        
        Args:
            N_predict (int): 预测时域长度
            
        Returns:
            tuple: 最低约束条件和最高约束条件
        """
        lbg = []  # 等式最低约束条件
        ubg = []  # 等式最高约束条件
        for _ in range(N_predict + 1):
            lbg.append(self.state_l)
            ubg.append(self.state_u)
        lbg = np.array(lbg).reshape(-1, 1)
        ubg = np.array(ubg).reshape(-1, 1)
        return lbg, ubg
   

   def lqr(A, B, Q, R):
        """ 求解无限视距离散时间 LQR 控制器
            x[k+1] = A x[k] + B u[k]
            u[k] = -K*x[k]
            cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]

        # Args:
            A, B: 线性系统矩阵
            Q, R: 状态矩阵和输入惩罚矩阵，均为正定矩阵

        # Returns:
            K: LQR 增益矩阵
            P: Riccati的解
            E: 闭环系统的特征值
        """

        P = np.array(scipy.linalg.solve_discrete_are(A, B, Q, R))
        K = -np.array(scipy.linalg.solve(R + B.T @ P @ B, B.T @ P @ A))
        eigenvalues, eigenvec = scipy.linalg.eig(A + B @ K)

        return K, P, eigenvalues

if __name__ == '__main__':
    control = FullCourseMPC_OP()