# -*- encoding: utf-8 -*-
import numpy as np


class Kalman(object):
    """Class used to estimate the state of a system.
    """

    def __init__(self, A, B, C, D, Q, R):
        """State space assumed to be discrete time.
        """
        # TODO: error checking here
        # check A is square
        self.A = A

        # check B.rows == A.rows
        self.B = B

        # check C.cols == A.cols
        self.C = C

        # check D.rows == C.rows
        # check D.cols == B.cols
        self.D = D

        # check Q is square
        # check Q.rows == A.rows
        self.Q = Q

        # check R is square
        # check R.cols == B.cols
        self.R = R

        self.P = None
        self.x = None
        self.K = None

        self.n = A.shape[0] # state size
        self.m = B.shape[1] # input size
        self.o = C.shape[0] # output size


    def estimate_state(self, u, z, P0=None, x0=None):
        """Calculates the state estimate over time given the input and
        measurement vector samples and an initial state estimate and error
        covariance matrix.

        u: input vector samples, ndarray with shape (N, m, 1)
        z: measurement vector samples, ndarray with shape (N, o, 1)
        P0: initial error covariance matrix, matrix of shape (n, n)
            (defaults to zero)
        x0: initial state estimate, vector of shape (n, 1)
            (defaults to zero)

        where
        n = size of state, A.rows()
        m = size of input, B.cols()
        p = size of measurement, C.rows()
        N = number of samples

        Returns the state estimate samples as an ndarray with shape (N, n, 1).
        """

        N = u.shape[0]
        x = np.zeros((N, self.n, 1))
        x_minus = np.zeros(x.shape)
        P = np.zeros((N, self.n, self.n))
        P_minus = np.zeros(P.shape)
        K = np.zeros((N, self.n, self.o))

        if P0 is None:
            P0 = np.zeros((self.n, self.n))
        if x0 is None:
            x0 = np.zeros((self.n, 1))

        # set P0, x0 as last elements in P, x
        # these will get overwritten
        P[-1, :] = P0
        x[-1, :] = x0
        I = np.eye(self.n)

        for k in range(N):
            ## time update (predict)
            # predicted state estimate
            # x_[k] = A*x[k] + B*u[k]
            x_minus[k, :] = (np.dot(self.A, x[k - 1, :]) +
                             np.dot(self.B, u[k, :]))
            # predicted estimate covariance
            # P_[k] = A*P[k - 1]*A' + Q
            P_minus[k, :] = (np.dot(np.dot(self.A, P[k - 1, :]), self.A.T) +
                             self.Q)

            # measurement update (update)
            # innovation covariance
            # S = C*P_[k]*C' + R
            S = np.dot(np.dot(self.C, P_minus[k, :]), self.C.T) + self.R
            # optimal Kalman gain
            # K[k] = P_[k]*C'*S^-1
            # K' = S^-1*C*P_[k].T
            # note: S is symmetric as all covariance matrices are symmetric
            K[k, :] = np.linalg.lstsq(S, np.dot(self.C, P_minus[k, :].T))[0].T
            # updated state estimate
            # x[k] = x_[k] + K[k]*(z[k] - C*x_[k])
            x[k, :] = (x_minus[k, :] +
                       np.dot(K[k, :], (z[k, :] - np.dot(self.C, x_minus[k, :]))))
            # updated estimate covariance
            # P[k] = (I - K[k]*C)*P_[k]
            P[k, :] = np.dot(I - np.dot(K[k, :], self.C), P_minus[k, :])
        return x
