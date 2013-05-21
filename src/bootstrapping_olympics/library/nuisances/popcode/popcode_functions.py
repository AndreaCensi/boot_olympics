from contracts import contract
import numpy as np
from bootstrapping_olympics.utils import assert_allclose

__all__ = ['popcode', 'popcode_inv']


@contract(y='array[N](>=0,<=1)',
          resolution='int,>1,M',
          returns='array[NxM](float32)')
def popcode(y, resolution):
    N = y.shape[0]
    pc = np.zeros((N, resolution), 'float32')

    for i in range(N):
        # assert 0 <= y[i] <= 1

        # y in [0, 1]
        x = y[i] * (resolution - 1)
        # x in [0, res-1]
        l = np.floor(x)  # l in [0, res-1]
        u = np.ceil(x)  # u in [0, res-1]

        if l == u:
            pc[i, l] = 1
            continue

        lr = x - l
        if False:
            ur = u - x
            assert_allclose(lr + ur, 1)

        pc[i, l] = 1 - lr
        pc[i, u] = lr

    return pc


@contract(z='array[NxM]',
          returns='array[N](>=0,<=1,float32)')
def popcode_inv(z):
    _, resolution = z.shape
    mult = np.linspace(0, 1, resolution)
    y = np.dot(z, mult)
    return y.astype('float32')









