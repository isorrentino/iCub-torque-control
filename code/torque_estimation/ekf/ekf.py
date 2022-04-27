from abc import abstractmethod
import numpy as np

class ExtendedKalmanFilter(object):

    """EKF abstract class"""

    def __init__(self, n, m, q, r, p0=0.001):

        """Initialize ExtendedKalmanFilter object

        Parameters:
        n : int
            state vector size
        m : int
            measurement vector size
        p0 : float
            prediction noise covariance
        q : float
            state noise covariance
        r : float
            measurement noise covariance
        """

        # No previous prediction noise covariance
        self.P_pre = None

        # Current state is zero, with diagonal noise covariance matrix
        self.x = np.zeros(n)
        self.P_post = np.eye(n) * p0

        # Set up covariance matrices for process noise and measurement noise
        self.Q = np.diag(q)
        self.R = np.diag(r)

    def step(self, z, **kwargs):
        """Runs one step of the EKF on observations z.
        This is the SW only implementation.
        Parameters
        ----------
        z: tuple
            A tuple of length m
        Returns
        -------
        numpy.ndarray
            representing the updated state.
        """

        # Predict $\hat{x}_k = f(\hat{x}_{k-1})$
        self.x, F = self.f(self.x, **kwargs)

        # $P_k = F_{k-1} P_{k-1} F^T_{k-1} + Q_{k-1}$
        self.P_pre = np.dot(np.dot(F, self.P_post), F.T) + self.Q

        # Update
        h, H = self.h(self.x, **kwargs)

        # $G_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1}$
        G = np.dot(np.dot(self.P_pre, H.T), np.linalg.inv(
            np.dot(np.dot(H, self.P_pre), H.T) + self.R))

        # $\hat{x}_k = \hat{x_k} + G_k(z_k - h(\hat{x}_k))$
        self.x += np.dot(G, (z - h))

        # $P_k = (I - G_k H_k) P_k$
        self.P_post = np.dot(self.I - np.dot(G, H), self.P_pre)

        return self.x

    @abstractmethod
    def f(self, x):
        """Abstract method for f(x,u)
        Your implementing class should define this method for the
        state-transition function f(x).
        Your state-transition function should return a NumPy array of n
        elements representing the new state, and a nXn NumPy array of elements representing the
        Jacobian of the function with respect to the new state.
        """
        raise NotImplementedError("Method f() is not implemented.")

    @abstractmethod
    def h(self, x):
        """Abstract method for h(x)
        Your implementing class should define this method for the
        observation function h(x), returning
        a NumPy array of m elements, and a NumPy array of m x n elements
        representing the Jacobian matrix
        H of the observation function with respect to the observation.
        """
        raise NotImplementedError("Method h() is not implemented.")


