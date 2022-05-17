# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np
import scipy.linalg


class UKF:

    def __init__(
            self,
            dim_x,
            dim_z,
            dt,
            fx,
            hx,
            process_noise,
            measurement_noise,
            alpha=0.001,
            beta=2,
            kappa=0
    ) -> None:
        self.x = np.zeros(dim_x)  # state, used for prediction and measurement
        self.P = np.eye(dim_x)  # Covariance, used for prediction and measurement

        self.x_prior = np.copy(self.x)  # Prior state estimate
        self.P_prior = np.copy(self.P)  # Prior state covariance matrix

        self.dt = dt  # sampling time

        self.fx = fx  # process function
        self.hx = hx  # output function

        self.Q = process_noise  # process noise
        self.R = measurement_noise  # measurement noise

        self.alpha_ = alpha  # alpha, used for calculating sigma points and associated weights
        self.beta_ = beta  # beta, used for calculating sigma points and associated weights
        self.k = kappa  # kappa, used for calculating sigma points and associated weights
        self.dim_x = dim_x  # dimension of the state
        self.dim_z = dim_z  # dimension of the measurement
        self.lambda_ = pow(self.alpha_, 2) * (self.dim_x + self.k) - self.dim_x

        self.Wc = self.__compute_weights()[0]  # weights to compute covariances
        self.Wm = self.__compute_weights()[1]  # weights to compute mean

        # all vectors used in the UKF process
        self.x_sigma_pts = np.zeros([2 * self.dim_x + 1, self.dim_x])  # sigma points
        self.f_sigma_pts = np.zeros([2 * self.dim_x + 1, self.dim_x])

    def __compute_weights(self):
        wc = np.full(2 * self.dim_x + 1, 1. / (2 * (self.dim_x + self.lambda_)))
        wm = np.full(2 * self.dim_x + 1, 1. / (2 * (self.dim_x + self.lambda_)))

        wc[0] = (self.lambda_ / (self.dim_x + self.lambda_)) + (1 - pow(self.alpha_, 2) + self.beta_)
        wm[0] = (self.lambda_ / (self.dim_x + self.lambda_))

        return wc, wm

    def reset_filter(self, x0, P0):
        self.x = x0
        self.P = P0

    @property
    def __update_sigma_points(self):
        """generates sigma points"""
        sqrt_matrix = scipy.linalg.cholesky((self.dim_x + self.lambda_) * self.P)

        self.x_sigma_pts[0] = self.x

        for i in range(self.dim_x):
            self.x_sigma_pts[i + 1] = self.x + sqrt_matrix[i]
            self.x_sigma_pts[i + 1 + self.dim_x] = self.x - sqrt_matrix[i]

    def compute_process_sigma_pts(self, sigma_pts_in, **fx_args):
        """
       Calculate the sigam points transformed the process function fx
       Input:
             input_sigma_pts: input sigma points
             **fx_args: keywords/arguments associated with process/system function defined as fx
       Output:
             output_sigma_pts: sigma points transformed by the process
       """
        n_pts = sigma_pts_in.shape[0]
        f_sigma_pts = np.zeros([n_pts, self.dim_x])

        for i, pt in enumerate(sigma_pts_in):
            f_sigma_pts[i] = self.fx(pt, self.dt, **fx_args)

        return f_sigma_pts

    def compute_mean_covariance(self, sigma_pts, M):
        """
        Utility functon to compute the mean and covariance in both the prediciton and update stages
        Inupt: sigma_pts: sigma points transfored by the process or the measurement
               M: process or measurement noise Matrix (Q or R)
        Out: mean: mean
             cov: covariance
        """
        predicted_mean = np.dot(self.Wm, sigma_pts)
        n_sigmas, n = sigma_pts.shape
        cov = np.zeros((n, n))
        for k in range(n_sigmas):
            err = (sigma_pts[k] - predicted_mean)
            err = err.reshape(n, 1)  # need to convert into 2D array, for the transpose operation !!!
            cov += self.Wc[k] * np.dot(err, err.T)
        cov += M
        return predicted_mean, cov

    def predict(self, **fx_args):
        """
        Prediction step. It computes the prior state and covariance
        :param fx_args: arguments associated with the process function defined by fx
        """
        # compute sigma points
        self.__update_sigma_points

        sigma_pts = self.x_sigma_pts

        self.f_sigma_pts = self.compute_process_sigma_pts(sigma_pts, **fx_args)

        self.x_prior, self.P_prior = self.compute_mean_covariance(self.f_sigma_pts, self.Q)

    def compute_measurement_sigma_pts(self, f_sigma_pts, **hx_args):
        """
        Compute the sigma points transformed by the output function h
        :param f_sigma_pts: input sigma points compute in the predict step
        :param hx_args: args of the output function
        :return: sigma points transformed by the output dynamics
        """
        n_pts = f_sigma_pts.shape[0]

        h_sigma_points = np.zeros([n_pts, self.dim_z])
        for i in range(n_pts):
            h_sigma_points[i] = self.hx(f_sigma_pts[i], **hx_args)

        return h_sigma_points

    def update(self, z: object, **hx_args: object):
        """
        Update step. It computes the posterior state and covariance
        :param z: measurement
        :param hx_args: arguments associated with the output function defined by hx
        """
        n_pts = self.x_sigma_pts.shape[0]

        self.h_sigma_pts = self.compute_measurement_sigma_pts(self.f_sigma_pts, **hx_args)

        # Compute predicted mean observation
        y_prior, Pyy = self.compute_mean_covariance(self.h_sigma_pts, self.R)

        # Compute cross covariances
        Pxy = np.zeros([self.dim_x, self.dim_z])
        for i in range(n_pts):
            x_err = self.f_sigma_pts[i] - self.x_prior
            y_err = self.h_sigma_pts[i] - y_prior
            Pxy += self.Wc[i] * np.outer(x_err, y_err)

        # Compute the kalman gain
        K = np.dot(Pxy, scipy.linalg.inv(Pyy))

        # New state and covariance estimate
        self.x = self.x_prior + np.dot(K, z - y_prior)
        self.P = self.P_prior - np.dot(K, Pyy).dot(K.T)

        return self.x
