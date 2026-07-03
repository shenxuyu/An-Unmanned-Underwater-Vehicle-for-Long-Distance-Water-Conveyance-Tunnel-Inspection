import numpy as np

# 加载自定义函数
from my_gp_class import MYGP



'''加载数据,需要使用绝对路径'''
r = np.load("r.npy")
Fr = np.load("Fr.npy")
Y = np.load("Y.npy")
Y = np.array(Y).reshape(-1, 1)  # 这里将 Y 转换为二维数组
Z = np.hstack([r, Fr])  # z = [x,u]
print(f"Z shape: {Z.shape}, Y shape: {Y.shape}") # 确保数据已正确加载
    
''' 超参数测试数据量'''
N_hyp = len(Z) // 3
print(f"N_hyp: {N_hyp}") # 打印数据形状以进行调试
    
'''用于GP超参数优化的训练集'''
X_hyp = Z[:N_hyp]
Y_hyp = Y[:N_hyp]
    
''' 用于生成GP预测模型的训练集'''
X_test = Z[N_hyp:]
Y_test = Y[N_hyp:]
    
''' 打印分割后的数据形状以进行调试'''
print(f"X_hyp shape: {X_hyp.shape}, Y_hyp shape: {Y_hyp.shape}")
print(f"X_test shape: {X_test.shape}, Y_test shape: {Y_test.shape}")
    
'''初始化GP模型'''
gp = MYGP(X_hyp, Y_hyp, X_test, Y_test)
print('=================')
print("GP模型初始化成功")
print('=================')