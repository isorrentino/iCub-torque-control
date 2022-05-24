import numpy as np
import matplotlib.pyplot as plt
from filters.ukf import ukf


def process(x_in, timestep, **inputs):
    """this function is based on the x_dot and can be nonlinear as needed"""
    ret = np.zeros(len(x_in))
    ret[0] = x_in[0] + timestep * np.cos(100*x_in[1])
    ret[1] = x_in[1] + timestep * x_in[2]
    ret[2] = x_in[2]
    return ret


def measurement(x_in):
    return x_in[1]


def main():
    np.set_printoptions(precision=3)

    # Process noise
    q = np.eye(3)
    q[0][0] = 0.001
    q[1][1] = 0.2
    q[2][2] = 0.0000001

    # Measurement noise
    r = 0.001

    x0 = np.array([-10.5, 1.8, 2.7])
    P0 = np.diag([5, 0.0255, 0.1])

    # Create UKF object
    observer = ukf.UKF(dim_x=3, dim_z=1, fx=process, hx=measurement, dt=0.001,
                       process_noise=q, measurement_noise=r,
                       alpha=.00001, beta=2, kappa=0)

    observer.reset_filter(x0=x0, P0=P0)

    x = np.copy(x0)

    z = np.zeros(100)
    x = np.zeros([3, 100])


    for i in range(100):
        observer.predict()
        z[i] = x[1, i] + i * 0.001
        x[:, i] = observer.update(z[i])

    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(x[0, :])
    plt.subplot(3, 1, 2)
    plt.plot(x[1, :])
    plt.subplot(3, 1, 3)
    plt.plot(x[2, :])
    plt.show()


if __name__ == "__main__":
    main()
