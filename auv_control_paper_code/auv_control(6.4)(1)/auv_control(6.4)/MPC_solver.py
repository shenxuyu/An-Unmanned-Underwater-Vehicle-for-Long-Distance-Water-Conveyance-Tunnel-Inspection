#===============建立MPC求解器==============
import math
import casadi as ca


def get_MPC_solver(state_fun, Nx, Nu, N_predict, Q, Q_end, R):
    """
    state_fun:      输入为离散状态方程;
           Nx:      状态量个数;
           Nu:      控制量个数;
    N_predict:      预测步长;
            Q:      状态惩罚矩阵;
        Q_end:      终端惩罚矩阵;
            R:      控制惩罚矩阵.
    """
    # 定义控制量和状态量
    u_all_horizon = ca.SX.sym('u_all_horizon', Nu, N_predict)        # N步内的控制输出
    x_all_horizon = ca.SX.sym('x_all_horizon', Nx, N_predict + 1)    # N+1步的系统状态，通常长度比控制多1
    p = ca.SX.sym('P', Nx, N_predict + 2)                            # 输入量，目前任务为轨迹跟踪，所以输入为初始状态与N+1步内全部的参考状态量，初始状态在前
    # 状态更新
    x_all_horizon[:, 0] = p[:, 0]
    for i in range(N_predict):
        x_all_horizon[:, i + 1] = state_fun(x_all_horizon[:, i], u_all_horizon[:, i])
    obj = 0
    # 构建损失函数(阶段+终端)
    for i in range(N_predict):
        obj = obj + ca.mtimes([(x_all_horizon[:, i]-p[:, i + 1]).T, Q, x_all_horizon[:, i]-p[:, i + 1]]) + ca.mtimes([u_all_horizon[:, i].T, R, u_all_horizon[:, i]])
    obj += ca.mtimes([(x_all_horizon[:, N_predict]-p[:, -1]).T, Q_end, x_all_horizon[:, N_predict]-p[:, -1]])
    # 采用single-shoot方法，这种方法在之前的仿真过程中速度要快于multiple-shoot法，但是可能会有稳定性的问题，之后可以尝试更换方法
    g = []
    for i in range(N_predict + 1):
        for j in range(Nx):
            g.append(x_all_horizon[j, i])
    # 定义NLP问题，'f'为目标函数，'x'为需寻找的优化结果（优化目标变量），'p'为系统参数，'g'为等式约束条件
    # 需要注意的是，用SX表达必须将所有表示成标量或者是一维矢量的形式
    nlp_prob = {'f': obj, 'x': ca.reshape(u_all_horizon, -1, 1), 'p':ca.reshape(p, -1, 1), 'g':ca.vertcat(*g)}
    #nlp_prob = {'f': obj, 'x': ca.reshape(u_all_horizon, -1, 1), 'p': ca.reshape(p, -1, 1)}
    # 优化选项设置
    opts_setting = {
        'ipopt.max_iter': 1000,
        'ipopt.print_level': 0,
        'print_time': 0,
        'ipopt.acceptable_tol': 1e-8,
        'ipopt.acceptable_obj_change_tol': 1e-6,
        'ipopt.tol': 1e-8,
    }
    # 最终目标，获得求解器
    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)
    # print(solver)
    return solver