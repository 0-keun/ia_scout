#!/usr/bin/env python3

import numpy as np

class MovAvg_Filter():

    def __init__(self):
        self.n = 10
        self.x_n = np.ones(self.n)

    def mov_avg_filter(self, x_meas):
        n = self.n
        if (self.x_n == np.ones(self.n)).all():
            self.x_n = x_meas*self.x_n
            self.x_avg = x_meas

        else:
            for i in range(n-1):
                self.x_n[i] = self.x_n[i+1]
            self.x_n[n-1] = x_meas
            self.x_avg = np.mean(self.x_n)

        return self.x_avg


class Kalman_Filter():
    def __init__(self):
        # Initialization for system model.
        self.A = 1
        self.H = 1
        self.Q = 0
        self.R = 4
        # Initialization for estimation.
        self.x_esti = 0.01  # 14 for book.
        self.P = 6


    def kalman_filter(self, z_meas):
        """Kalman Filter Algorithm for One Variable."""
        # Prediction.
        x_pred = self.A * self.x_esti
        P_pred = self.A * self.P * self.A + self.Q

        # Kalman Gain.
        K = P_pred * self.H / (self.H * P_pred * self.H + self.R)

        # Estimation.
        self.x_esti = x_pred + K * (z_meas - self.H * x_pred)

        # Error Covariance.
        self.P = P_pred - K * self.H * P_pred

        return self.x_esti

