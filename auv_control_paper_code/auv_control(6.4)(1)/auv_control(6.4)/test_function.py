import casadi as ca

def ode_error_yaw(x, u, feedback_r, pre_feedback_dvl_x, pre_feedback_dvl_y):
    """误差方程"""
    dxdt = [feedback_r - (u / -0.341607 - 13.955754 * pre_feedback_dvl_x * pre_feedback_dvl_y + 0.149469 * x - 0.021017 * ca.fabs(x) * x)]
    return dxdt

def get_ode_ca_distrub(ode, Nx, Nu, feedback_r, pre_feedback_dvl_x, pre_feedback_dvl_y):
    """将list形式的ode表达式构建成casadi表达式"""
    X = ca.SX.sym('x', Nx)
    U = ca.SX.sym('u', Nu)
    ode_expr = ode(X, U, feedback_r, pre_feedback_dvl_x, pre_feedback_dvl_y)
    ode_ca = ca.Function("ode_ca", [X, U], [ca.vertcat(*ode_expr)])
    return ode_ca

Nx = 1
Nu = 1
pre_feedback_dvl_x = 1.5
pre_feedback_dvl_y = 2
feedback_r = 3

ode_nominal_ca = get_ode_ca_distrub(ode_error_yaw, Nx, Nu, feedback_r, pre_feedback_dvl_x, pre_feedback_dvl_y)

# To test the function:
x_test = ca.DM([1])
u_test = ca.DM([2])
result = ode_nominal_ca(x_test, u_test)
print(result)
