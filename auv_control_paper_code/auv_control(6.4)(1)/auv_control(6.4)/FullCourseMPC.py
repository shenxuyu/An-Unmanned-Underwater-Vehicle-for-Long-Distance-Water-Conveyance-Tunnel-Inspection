#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RSGP-MPC heading controller used in the UUV tunnel-inspection study.

This module contains the controller implementation used for heading tracking.
The controller combines a nominal yaw-rate model with a Gaussian-process
disturbance model and solves a finite-horizon model predictive control problem
using CasADi/IPOPT.

Required training-data files
----------------------------
The following files are loaded from ``training_data_dir``. If
``training_data_dir`` is not provided, the directory containing this file is
used.

    r.npy   : measured yaw-rate samples
    Fr.npy  : corresponding yaw-control input samples
    Y.npy   : model-error samples used as GP targets

The code is written to be independent of user-specific absolute paths so that
it can be shared as supplementary software.
"""

import time
from pathlib import Path

import casadi as ca
import numpy as np

try:
    from my_gp_class import MYGP
except ImportError:  # fallback for users who rename the file
    from m_gp_class import MYGP


class FullCourseMPC:
    """Recursive sparse GP-based MPC controller for UUV heading control."""

    def __init__(self, sample_time=0.2, training_data_dir=None, verbose=False):
        self.sample_time = float(sample_time)
        self.current_time = time.time()
        self.last_time = self.current_time
        self.verbose = bool(verbose)

        if training_data_dir is None:
            self.training_data_dir = Path(__file__).resolve().parent
        else:
            self.training_data_dir = Path(training_data_dir).expanduser().resolve()

        # GP model. It is initialised lazily when the controller is first used.
        self.gp = None

        # State and input dimensions.
        # The dynamic state is yaw rate r. The extended MPC state is [yaw, r].
        self.Nx = 1
        self.Nx_ext = 2 * self.Nx
        self.Nu = 1

        # MPC parameters.
        self.N_predict = 5
        self.Q = np.eye(self.Nx_ext) * 1e-1
        self.Q[0, 0] = 10.0
        self.Q[1, 1] = 0.1
        self.Q_end = 2.0 * np.eye(self.Nx_ext)
        self.R = np.zeros((self.Nu, self.Nu))

        # State constraints: yaw angle and yaw rate.
        self.state_l = np.array([-180.0, -30.0])
        self.state_u = np.array([180.0, 30.0])

        # Control constraint: yaw-control torque.
        self.control_l = np.array([-117.0])
        self.control_u = np.array([117.0])

        # Initial state and warm-start control sequence.
        self.State_Initial = np.array([0.0, 0.0])
        self.Control_init = np.zeros((self.N_predict, self.Nu))
        self.output = 0.0

        # Reference trajectory over the prediction horizon.
        self.state_ref = np.zeros((self.Nx_ext, self.N_predict + 1))

        self.clear()

    def clear(self):
        """Reset the controller reference and output."""
        self.exp_course = 0.0
        self.exp_r = 0.0
        self.output = 0.0
        self.Control_init = np.zeros((self.N_predict, self.Nu))

    # ------------------------------------------------------------------
    # GP initialisation
    # ------------------------------------------------------------------
    def _load_gp_training_data(self):
        """Load GP training data from relative files."""
        r_path = self.training_data_dir / "r.npy"
        fr_path = self.training_data_dir / "Fr.npy"
        y_path = self.training_data_dir / "Y.npy"

        missing = [str(p) for p in (r_path, fr_path, y_path) if not p.exists()]
        if missing:
            raise FileNotFoundError(
                "Missing GP training-data file(s): " + ", ".join(missing)
            )

        r = np.asarray(np.load(str(r_path)), dtype=float).reshape(-1, 1)
        fr = np.asarray(np.load(str(fr_path)), dtype=float).reshape(-1, 1)
        y = np.asarray(np.load(str(y_path)), dtype=float).reshape(-1, 1)

        n = min(r.shape[0], fr.shape[0], y.shape[0])
        if n < 2:
            raise ValueError("At least two GP training samples are required.")

        z = np.hstack([r[:n], fr[:n]])
        y = y[:n]

        valid = np.all(np.isfinite(z), axis=1) & np.all(np.isfinite(y), axis=1)
        z = z[valid]
        y = y[valid]

        if z.shape[0] < 2:
            raise ValueError("The GP training data contain insufficient valid samples.")

        return z, y

    def _initialise_gp(self):
        """Initialise the GP model once using the available training data."""
        if self.gp is not None:
            return

        z, y = self._load_gp_training_data()

        # One subset is used for hyperparameter optimisation; the remaining
        # samples are used to build the GP prediction function.
        n_hyp = max(1, z.shape[0] // 3)
        if n_hyp >= z.shape[0]:
            n_hyp = max(1, z.shape[0] // 2)

        x_hyp = z[:n_hyp]
        y_hyp = y[:n_hyp]
        x_train = z[n_hyp:]
        y_train = y[n_hyp:]

        if x_train.shape[0] == 0:
            x_train = z
            y_train = y

        self.gp = MYGP(x_hyp, y_hyp, x_train, y_train, verbose=self.verbose)

        if self.verbose:
            print("GP model initialised with {} training samples.".format(z.shape[0]))

    # ------------------------------------------------------------------
    # MPC solver
    # ------------------------------------------------------------------
    def get_MPC_solver(self, state_fun, Nx, Nu, N_predict, Q, Q_end, R):
        """
        Build the single-shooting MPC solver.

        Parameters
        ----------
        state_fun : casadi.Function
            Discrete state-transition function.
        Nx : int
            Number of states.
        Nu : int
            Number of control inputs.
        N_predict : int
            Prediction horizon.
        Q, Q_end, R : ndarray
            Stage-state, terminal-state and control-weight matrices.
        """
        u_all = ca.SX.sym("u_all", Nu, N_predict)
        x_all = ca.SX.sym("x_all", Nx, N_predict + 1)

        # Parameter matrix: initial state followed by N+1 reference states.
        p = ca.SX.sym("P", Nx, N_predict + 2)

        x_all[:, 0] = p[:, 0]
        for k in range(N_predict):
            x_all[:, k + 1] = state_fun(x_all[:, k], u_all[:, k])

        obj = 0
        for k in range(N_predict):
            state_error = x_all[:, k] - p[:, k + 1]
            obj += ca.mtimes([state_error.T, Q, state_error])
            obj += ca.mtimes([u_all[:, k].T, R, u_all[:, k]])

        terminal_error = x_all[:, N_predict] - p[:, -1]
        obj += ca.mtimes([terminal_error.T, Q_end, terminal_error])

        # State constraints are imposed through the predicted states.
        g = []
        for k in range(N_predict + 1):
            for j in range(Nx):
                g.append(x_all[j, k])

        nlp_prob = {
            "f": obj,
            "x": ca.reshape(u_all, -1, 1),
            "p": ca.reshape(p, -1, 1),
            "g": ca.vertcat(*g),
        }

        opts = {
            "ipopt.max_iter": 1000,
            "ipopt.print_level": 0,
            "print_time": 0,
            "ipopt.acceptable_tol": 1e-8,
            "ipopt.acceptable_obj_change_tol": 1e-6,
            "ipopt.tol": 1e-8,
        }

        return ca.nlpsol("solver", "ipopt", nlp_prob, opts)

    def mpc_update(self, pre_feedbackcourse, pre_feedback_r,
                   pre_feedback_dvl_x, pre_feedback_dvl_y):
        """
        Solve one MPC step and update ``self.output``.

        Parameters
        ----------
        pre_feedbackcourse : float
            Current yaw angle in degrees.
        pre_feedback_r : float
            Current yaw rate.
        pre_feedback_dvl_x, pre_feedback_dvl_y : float
            DVL velocity components used by the nominal yaw-rate model.
        """
        self._initialise_gp()

        dt = self.sample_time
        self.current_time = time.time()
        self.last_time = self.current_time

        ode_nominal_ca = self.get_ode_ca(
            self.ode_nominal_yaw,
            self.Nx,
            self.Nu,
            pre_feedback_dvl_x,
            pre_feedback_dvl_y,
        )

        gp_error_fun_ca = self.gp.get_mean_fun()
        corrected_dynamics = self.merge_model(
            ode_nominal_ca,
            gp_error_fun_ca,
            self.Nx,
            self.Nu,
        )

        kinematic_model = self.dyna_2_kine(corrected_dynamics, self.Nx, self.Nu)
        discrete_model = self.my_rk4_fun(
            kinematic_model,
            dt,
            self.Nx_ext,
            self.Nu,
        )

        solver = self.get_MPC_solver(
            discrete_model,
            self.Nx_ext,
            self.Nu,
            self.N_predict,
            self.Q,
            self.Q_end,
            self.R,
        )

        lbx = np.tile(self.control_l, self.N_predict).reshape(-1, 1)
        ubx = np.tile(self.control_u, self.N_predict).reshape(-1, 1)
        lbg = np.tile(self.state_l, self.N_predict + 1).reshape(-1, 1)
        ubg = np.tile(self.state_u, self.N_predict + 1).reshape(-1, 1)

        self.State_Initial[0] = float(pre_feedbackcourse)
        self.State_Initial[1] = float(pre_feedback_r)

        self.x_init = self.State_Initial.reshape(-1, 1)
        self.u_init = np.asarray(self.Control_init, dtype=float).reshape(
            self.N_predict,
            self.Nu,
        )

        if not np.all(np.isfinite(self.x_init)):
            raise ValueError("The initial state contains non-finite values.")
        if not np.all(np.isfinite(self.u_init)):
            raise ValueError("The warm-start control sequence contains non-finite values.")

        self.state_ref = self.get_mpc_yaw_ref(self.N_predict)
        if not np.all(np.isfinite(self.state_ref)):
            raise ValueError("The MPC reference trajectory contains non-finite values.")

        c_p = np.concatenate((self.x_init, self.state_ref), axis=1).T.reshape((-1, 1))
        init_control = ca.reshape(self.u_init, -1, 1)

        start_time = time.time()
        res = solver(
            x0=init_control,
            p=c_p,
            lbg=lbg,
            lbx=lbx,
            ubg=ubg,
            ubx=ubx,
        )
        solve_time = time.time() - start_time

        u_sol = np.asarray(ca.reshape(res["x"], self.Nu, self.N_predict).T, dtype=float)
        self.u_sol = u_sol
        self.Control_init = u_sol

        u0 = float(u_sol[0, 0])
        self.output = float(np.clip(u0, self.control_l[0], self.control_u[0]))

        # Store the state-control pair used for online GP updating.
        self.x_sub = np.asarray(self.x_init[1:], dtype=float).reshape(1, -1)
        self.u_sub = np.asarray([[self.output]], dtype=float)

        if self.verbose:
            print("MPC solve time: {:.6f} s".format(solve_time))
            print("MPC output: {:.6f}".format(self.output))

    def gp_model_update(self, feedback_r, pre_feedback_dvl_x, pre_feedback_dvl_y):
        """Update the GP training set using the latest yaw-rate error."""
        if self.gp is None:
            return

        ode_error_ca = self.get_ode_ca_distrub(
            self.ode_error_yaw,
            self.Nx,
            self.Nu,
            feedback_r,
            pre_feedback_dvl_x,
            pre_feedback_dvl_y,
        )

        error_this = ode_error_ca(self.x_sub.T, self.u_sub.T)
        y_data = np.asarray(error_this, dtype=float).reshape(1, -1)
        x_data = np.hstack((self.x_sub, self.u_sub))

        self.gp.data_update_new(x_data, y_data, recompute=True)

    # ------------------------------------------------------------------
    # Reference generation
    # ------------------------------------------------------------------
    def set_course(self, exp_course, feedbackcourse):
        """Set the desired yaw angle and yaw-rate reference."""
        self.exp_course = float(exp_course)
        yaw_error = self.angle_error_deg(self.exp_course, float(feedbackcourse))
        self.exp_r = yaw_error / self.sample_time

    def get_mpc_yaw_ref(self, N_predict):
        """Return the reference trajectory over the prediction horizon."""
        state_ref = np.empty((self.Nx_ext, N_predict + 1))
        state_ref[0, :] = self.exp_course
        state_ref[1, :] = self.exp_r
        return state_ref

    # ------------------------------------------------------------------
    # Yaw dynamics and GP-corrected model
    # ------------------------------------------------------------------
    def ode_nominal_yaw(self, x, u, feedback_vx, feedback_vy):
        """Nominal yaw-rate dynamics."""
        dxdt = [
            u / -0.34
            - 13.96 * feedback_vx * feedback_vy
            + 0.15 * x
            - 0.02 * x * ca.fabs(x)
        ]
        return dxdt

    def ode_error_yaw(self, x, u, feedback_r, pre_feedback_dvl_x,
                      pre_feedback_dvl_y):
        """Yaw-rate model-error equation used for online GP updating."""
        nominal = (
            u / -0.34
            - 13.96 * pre_feedback_dvl_x * pre_feedback_dvl_y
            + 0.15 * x
            - 0.02 * ca.fabs(x) * x
        )
        return [feedback_r - nominal]

    def merge_model(self, nominal_fun, error_fun, Nx, Nu):
        """Combine the nominal model and the GP-predicted model error."""
        x = ca.SX.sym("x", Nx)
        if Nu != 0:
            u = ca.SX.sym("u", Nu)
            return ca.Function(
                "corrected_dynamics",
                [x, u],
                [nominal_fun(x, u) + error_fun(x, u)],
            )

        return ca.Function("corrected_dynamics", [x], [nominal_fun(x) + error_fun(x)])

    def get_ode_ca_distrub(self, ode, Nx, Nu, feedback_r,
                            pre_feedback_dvl_x, pre_feedback_dvl_y):
        """Convert a disturbance/error ODE expression to a CasADi function."""
        x = ca.SX.sym("x", Nx)
        u = ca.SX.sym("u", Nu)
        ode_expr = ode(x, u, feedback_r, pre_feedback_dvl_x, pre_feedback_dvl_y)
        return ca.Function("ode_error", [x, u], [ca.vertcat(*ode_expr)])

    def get_ode_ca(self, ode, Nx, Nu, pre_feedback_vx, pre_feedback_vy):
        """Convert the nominal ODE expression to a CasADi function."""
        x = ca.SX.sym("x", Nx)
        u = ca.SX.sym("u", Nu)
        ode_expr = ode(x, u, pre_feedback_vx, pre_feedback_vy)
        return ca.Function("ode_nominal", [x, u], [ca.vertcat(*ode_expr)])

    def dyna_2_kine(self, ode_dyna, Nx, Nu):
        """Extend yaw-rate dynamics to the kinematic state [yaw, r]."""
        x_ext = ca.SX.sym("x_ext", 2 * Nx)
        u = ca.SX.sym("u", Nu)

        dx_ext = ca.vertcat(
            x_ext[Nx:],
            ode_dyna(x_ext[Nx:], u),
        )
        return ca.Function("kinematic_model", [x_ext, u], [dx_ext])

    def my_rk4_fun(self, ode, dt, Nx, Nu):
        """Discretise a continuous model using fourth-order Runge-Kutta."""
        x = ca.SX.sym("x", Nx)
        u = ca.SX.sym("u", Nu)

        k1 = ode(x, u)
        k2 = ode(x + dt / 2.0 * k1, u)
        k3 = ode(x + dt / 2.0 * k2, u)
        k4 = ode(x + dt * k3, u)

        x_next = x + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        return ca.Function("rk4_step", [x, u], [x_next])

    # ------------------------------------------------------------------
    # Angle utilities
    # ------------------------------------------------------------------
    @staticmethod
    def wrap_angle_deg(angle):
        """Wrap an angle to [-180, 180) degrees."""
        return (angle + 180.0) % 360.0 - 180.0

    @classmethod
    def angle_error_deg(cls, target, current):
        """Return the wrapped angular error target-current in degrees."""
        return cls.wrap_angle_deg(target - current)


if __name__ == "__main__":
    controller = FullCourseMPC(verbose=True)
    controller.set_course(0.5, 0.5)
    controller.mpc_update(1.0, 1.0, 1.0, 1.0)
    print("Control output:", controller.output)
