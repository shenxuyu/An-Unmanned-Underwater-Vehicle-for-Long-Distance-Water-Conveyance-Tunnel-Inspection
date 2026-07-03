import numpy as np
import casadi as ca
import pandas as pd


# 名义动力学模型函数定义
def ode_nominal_yaw(x, u, feedback_vx, feedback_vy):
    """
    名义动力学模型
    x: yaw 角速度 (r)
    u: z 轴转动力矩
    feedback_vx: 反馈的 vx
    feedback_vy: 反馈的 vy
    """
    dxdt = u / -0.341607 - 13.955754 * feedback_vx * feedback_vy + 0.149469 * x - 0.021017 * x * ca.fabs(x)
    return dxdt


def my_generate_training_data(file_path):
    # 加载 Excel 文件
    df = pd.read_excel(file_path)
    # 从 Excel 文件中提取数据
    yaw = df.iloc[:, 0].tolist()  # 第一列为 yaw
    r = df.iloc[:, 1].tolist()    # 第二列为 r
    u = df.iloc[:, 2].tolist()    # 第三列为 u
    v = df.iloc[:, 3].tolist()    # 第四列为 v
    Fr = df.iloc[:, 4].tolist()   # 第五列为 Fr

    # 初始化列表以存储计算结果
    data_nominal_r = []
    data_error_r = []

    # 计算名义动力学数据和误差
    for i in range(len(r) - 1):  # 循环到倒数第二个元素
        nominal_r = ode_nominal_yaw(r[i], u[i], v[i], Fr[i])
        data_nominal_r.append(nominal_r)
        error_r = r[i + 1] - nominal_r
        data_error_r.append(error_r)

    # 处理最后一个 r 值
    nominal_r = ode_nominal_yaw(r[-1], u[-1], v[-1], Fr[-1])
    data_nominal_r.append(nominal_r)

    # 将数据转换为 NumPy 数组
    Y = np.array(data_error_r).reshape(-1, 1)  # 这里将 Y 转换为二维数组

    # 确保 r 和 Fr 的大小与 Y 匹配
    a = np.size(Y, 0)
    r = np.array(r[:a]).reshape(-1, 1)
    Fr = np.array(Fr[:a]).reshape(-1, 1)

    # 水平堆叠 r 和 Fr
    Z = np.hstack([r, Fr])
    return Z, Y