# -*- coding: utf-8 -*-
"""
Gaussian-process model used by the RSGP-MPC heading controller.

The GP learns the residual between the nominal yaw-rate model and the measured
yaw-rate response. A squared-exponential ARD kernel is used. The prediction
function is exported as a CasADi expression so that it can be embedded directly
in the MPC optimisation problem.
"""

import time

import casadi as ca
import numpy as np
from scipy.optimize import minimize


def _as_2d(array, name):
    """Return ``array`` as a finite two-dimensional NumPy array."""
    value = np.asarray(array, dtype=float)
    if value.ndim == 1:
        value = value.reshape(-1, 1)
    if value.ndim != 2:
        raise ValueError("{} must be a one- or two-dimensional array.".format(name))
    if not np.all(np.isfinite(value)):
        raise ValueError("{} contains non-finite values.".format(name))
    return value


def my_covSEard(x_train, x, u, ell, sf2, N):
    """
    Squared-exponential ARD covariance between training inputs and a test input.

    Parameters
    ----------
    x_train : casadi.SX
        Training inputs with shape (N, state_dim + input_dim).
    x, u : casadi.SX
        Test state and control input.
    ell : casadi.SX
        ARD length scales.
    sf2 : casadi.SX
        Signal variance.
    N : int
        Number of training samples.
    """
    z = ca.vertcat(x, u).T
    z_predict = ca.repmat(z, N, 1)
    ell_repeated = ca.repmat(ell, N, 1)
    squared_distance = ca.sum2((x_train - z_predict) ** 2 / ell_repeated ** 2)
    return sf2 * ca.exp(-0.5 * squared_distance)


def calc_cov_matrix(X, ell, sf2):
    """
    Compute the squared-exponential ARD covariance matrix.

    Parameters
    ----------
    X : ndarray, shape (N, D)
        Training inputs.
    ell : ndarray, shape (D,)
        Positive length scales.
    sf2 : float
        Signal variance.
    """
    X = _as_2d(X, "X")
    ell = np.asarray(ell, dtype=float).reshape(1, -1)
    if ell.shape[1] != X.shape[1]:
        raise ValueError("The number of length scales must match X.shape[1].")
    if np.any(ell <= 0):
        raise ValueError("All GP length scales must be positive.")

    X_scaled = X / ell
    squared_norm = np.sum(X_scaled ** 2, axis=1).reshape(-1, 1)
    squared_distance = squared_norm + squared_norm.T - 2.0 * np.dot(X_scaled, X_scaled.T)
    squared_distance = np.maximum(squared_distance, 0.0)
    return float(sf2) * np.exp(-0.5 * squared_distance)


def calc_NLL_numpy(hyper, X, Y):
    """
    Negative log marginal likelihood used for GP hyperparameter optimisation.

    ``hyper`` contains ``[ell_1, ..., ell_D, sf, sn]``. The covariance matrix
    is ``K = K_SE + sn^2 I``.
    """
    X = _as_2d(X, "X")
    Y = _as_2d(Y, "Y")

    n, dim = X.shape
    hyper = np.asarray(hyper, dtype=float)

    ell = hyper[:dim]
    sf2 = hyper[dim] ** 2
    sn2 = hyper[dim + 1] ** 2

    K = calc_cov_matrix(X, ell, sf2)
    K = K + sn2 * np.eye(n)
    K = 0.5 * (K + K.T)

    jitter = 1e-10
    for _ in range(8):
        try:
            L = np.linalg.cholesky(K + jitter * np.eye(n))
            break
        except np.linalg.LinAlgError:
            jitter *= 10.0
    else:
        raise np.linalg.LinAlgError("The GP covariance matrix is not positive definite.")

    inv_L_y = np.linalg.solve(L, Y)
    alpha = np.linalg.solve(L.T, inv_L_y)

    log_det_K = 2.0 * np.sum(np.log(np.abs(np.diag(L))))
    nll = 0.5 * np.dot(Y.T, alpha) + 0.5 * log_det_K
    nll += 0.5 * n * np.log(2.0 * np.pi)

    return float(np.squeeze(nll))


class MYGP:
    """Gaussian-process residual model for RSGP-MPC."""

    def __init__(self, X_hyper_opt, Y_hyper_opt, x_train, y_train,
                 train_maxsize=4000, verbose=False):
        self.__X_hyper_opt = _as_2d(X_hyper_opt, "X_hyper_opt")
        self.__Y_hyper_opt = _as_2d(Y_hyper_opt, "Y_hyper_opt")
        self.x_train = _as_2d(x_train, "x_train")
        self.y_train = _as_2d(y_train, "y_train")

        if self.__X_hyper_opt.shape[0] != self.__Y_hyper_opt.shape[0]:
            raise ValueError("X_hyper_opt and Y_hyper_opt must have the same length.")
        if self.x_train.shape[0] != self.y_train.shape[0]:
            raise ValueError("x_train and y_train must have the same length.")

        self.__Ny = self.__Y_hyper_opt.shape[1]
        self.__Nx = self.__X_hyper_opt.shape[1]
        self.__N = self.__X_hyper_opt.shape[0]
        self.__Nu = self.__Nx - self.__Ny

        if self.__Nu <= 0:
            raise ValueError(
                "The GP input dimension must be larger than the output dimension. "
                "Expected inputs to contain state and control variables."
            )
        if self.x_train.shape[1] != self.__Nx:
            raise ValueError("x_train must have the same input dimension as X_hyper_opt.")
        if self.y_train.shape[1] != self.__Ny:
            raise ValueError("y_train must have the same output dimension as Y_hyper_opt.")

        self.train_maxsize = int(train_maxsize)
        self.train_originsize = self.x_train.shape[0]
        self.verbose = bool(verbose)

        self.hyp_opt = None
        self.invK = None
        self.alpha = None
        self.chol = None

        self.optimize()
        self.get_predict_parameter()

    def optimize(self):
        """Optimise GP hyperparameters by minimising the negative log likelihood."""
        num_ell = self.__Nx
        num_hyp = num_ell + 2

        self.hyp_opt = np.zeros((self.__Ny, num_hyp))
        options = {"disp": self.verbose, "maxiter": 10000}

        if self.verbose:
            print("\n________________________________________")
            print("# Optimising GP hyperparameters (N={})".format(self.__N))
            print("----------------------------------------")

        for output in range(self.__Ny):
            lower = np.full(num_hyp, -np.inf)
            upper = np.full(num_hyp, np.inf)

            lower[:self.__Nx] = 1e-2
            upper[:self.__Nx] = 2e2
            lower[self.__Nx] = 1e-8
            upper[self.__Nx] = 1e2
            lower[self.__Nx + 1] = 1e-10
            upper[self.__Nx + 1] = 1e-2
            bounds = list(zip(lower, upper))

            hyp_init = np.zeros(num_hyp)
            hyp_init[:self.__Nx] = np.maximum(
                np.std(self.__X_hyper_opt, axis=0),
                1e-2,
            )
            hyp_init[self.__Nx] = max(np.std(self.__Y_hyper_opt[:, output]), 1e-6)
            hyp_init[self.__Nx + 1] = 1e-5

            start_time = time.time()
            res = minimize(
                calc_NLL_numpy,
                hyp_init,
                args=(self.__X_hyper_opt, self.__Y_hyper_opt[:, output]),
                method="SLSQP",
                options=options,
                bounds=bounds,
                tol=1e-12,
            )
            solve_time = time.time() - start_time

            if not res.success and self.verbose:
                print("Warning: GP hyperparameter optimisation did not converge.")
                print("Message:", res.message)

            self.hyp_opt[output, :] = res.x

            if self.verbose:
                print("* Output {}: {:.6f} s".format(output, solve_time))

        if self.verbose:
            print("----------------------------------------")

    def get_predict_parameter(self):
        """Compute matrices required by the GP posterior mean."""
        n_train = self.x_train.shape[0]

        self.invK = np.zeros((self.__Ny, n_train, n_train))
        self.alpha = np.zeros((self.__Ny, n_train))
        self.chol = np.zeros((self.__Ny, n_train, n_train))

        for output in range(self.__Ny):
            ell = self.hyp_opt[output, :self.__Nx]
            sf2 = self.hyp_opt[output, self.__Nx] ** 2
            sn2 = self.hyp_opt[output, self.__Nx + 1] ** 2

            K = calc_cov_matrix(self.x_train, ell, sf2)
            K = K + sn2 * np.eye(n_train)
            K = 0.5 * (K + K.T)

            jitter = 1e-10
            for _ in range(8):
                try:
                    L = np.linalg.cholesky(K + jitter * np.eye(n_train))
                    break
                except np.linalg.LinAlgError:
                    jitter *= 10.0
            else:
                raise np.linalg.LinAlgError(
                    "The GP covariance matrix is not positive definite."
                )

            inv_L = np.linalg.solve(L, np.eye(n_train))
            self.invK[output, :, :] = np.linalg.solve(L.T, inv_L)
            self.chol[output, :, :] = L
            self.alpha[output, :] = np.linalg.solve(
                L.T,
                np.linalg.solve(L, self.y_train[:, output]),
            )

    def get_mean_fun(self):
        """
        Return the GP posterior mean as a CasADi function.

        The returned function has the signature ``gp_mean(x, u)`` and returns
        the predicted residual for the yaw-rate model.
        """
        n_train = self.x_train.shape[0]

        x = ca.SX.sym("x", self.__Ny)
        u = ca.SX.sym("u", self.__Nu)
        x_train_ca = ca.SX(self.x_train)

        outputs = []
        for output in range(self.__Ny):
            ell = ca.SX(self.hyp_opt[output, :self.__Nx].reshape(1, -1))
            sf2 = ca.SX(self.hyp_opt[output, self.__Nx] ** 2)
            alpha = ca.SX(self.alpha[output, :].reshape(-1, 1))

            covariance = my_covSEard(x_train_ca, x, u, ell, sf2, n_train)
            outputs.append(ca.mtimes(covariance.T, alpha))

        return ca.Function("gp_mean", [x, u], [ca.vertcat(*outputs)])

    def data_update(self, x_in, y_in):
        """Update the GP dataset by replacing the oldest sample."""
        x_in = _as_2d(x_in, "x_in")
        y_in = _as_2d(y_in, "y_in")

        if x_in.shape[0] != 1 or y_in.shape[0] != 1:
            raise ValueError("data_update expects one input-output sample.")
        if x_in.shape[1] != self.x_train.shape[1]:
            raise ValueError("x_in has an unexpected number of columns.")
        if y_in.shape[1] != self.y_train.shape[1]:
            raise ValueError("y_in has an unexpected number of columns.")

        self.x_train[:-1, :] = self.x_train[1:, :]
        self.x_train[-1, :] = x_in[0, :]

        self.y_train[:-1, :] = self.y_train[1:, :]
        self.y_train[-1, :] = y_in[0, :]

    def data_update_new(self, x_in, y_in, recompute=False):
        """
        Append a new sample to the GP dataset.

        When the maximum dataset size is reached, the original offline samples
        are retained and the oldest online sample is replaced. If ``recompute``
        is True, posterior matrices are updated immediately.
        """
        x_in = _as_2d(x_in, "x_in")
        y_in = _as_2d(y_in, "y_in")

        if x_in.shape[0] != 1 or y_in.shape[0] != 1:
            raise ValueError("data_update_new expects one input-output sample.")
        if x_in.shape[1] != self.x_train.shape[1]:
            raise ValueError("x_in has an unexpected number of columns.")
        if y_in.shape[1] != self.y_train.shape[1]:
            raise ValueError("y_in has an unexpected number of columns.")

        n_train = self.x_train.shape[0]
        if n_train < self.train_maxsize:
            self.x_train = np.vstack((self.x_train, x_in))
            self.y_train = np.vstack((self.y_train, y_in))
        else:
            start = min(self.train_originsize, n_train - 1)
            self.x_train[start:-1, :] = self.x_train[start + 1:, :]
            self.x_train[-1, :] = x_in[0, :]

            self.y_train[start:-1, :] = self.y_train[start + 1:, :]
            self.y_train[-1, :] = y_in[0, :]

        if recompute:
            self.get_predict_parameter()

    def mean_fun_update(self):
        """Update posterior matrices and return the updated GP mean function."""
        self.get_predict_parameter()
        return self.get_mean_fun()

    def get_all_train_data(self):
        """Return the current GP training inputs and targets."""
        return self.x_train.copy(), self.y_train.copy()
