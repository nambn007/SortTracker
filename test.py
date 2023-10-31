import numpy as np 
from filterpy.kalman import KalmanFilter


def reshape_z(z, dim_z, ndim):
    """ ensure z is a (dim_z, 1) shaped vector"""

    z = np.atleast_2d(z)
    if z.shape[1] == dim_z:
        z = z.T

    if z.shape != (dim_z, 1):
        raise ValueError('z must be convertible to shape ({}, 1)'.format(dim_z))

    if ndim == 1:
        z = z[:, 0]

    if ndim == 0:
        z = z[0, 0]

    return z


x = np.array([1, 2, 3, 4]).reshape((4, 1))
print(x)
x = reshape_z(x, 4, 7)
print(x)