from . import np
from .popcode_functions import popcode, popcode_inv
from bootstrapping_olympics.utils import assert_allclose


def test_popcode1():
    resolution = 10
    N = 5
    for resolution in [2, 5, 10, 20]:
        a = np.random.rand(N)
        b = popcode(a, resolution=resolution)
        a2 = popcode_inv(b)

        # print('---')
        # print(' a = %s' % a)
        # print(' b = %s' % b)
        # print('a2 = %s' % a2)

        assert b.shape == (N, resolution)
        assert a2.shape == a.shape
        assert_allclose(a2, a)


examples = [
    # res, a, popcode(a)
    (3, 0, [1, 0, 0]),
    # res, a, popcode(a)
    (3, 1, [0, 0, 1]),
    (3, .5, [0, 1, 0]),
    (4, .5, [0, .5, .5, 0]),
]


def test_popcode():
    for resolution, a, b in examples:
        a = np.array([a]).astype('float32')
        b2 = popcode(a, resolution)
        #print(' a = %s' % a)
        #print(' b = %s' % b)
        #print('b2 = %s' % b2)
        assert_allclose(b2[0], b)


def test_popcode_inv():
    for _, a, b in examples:
        b = np.array([b]).astype('float32')
        a2 = popcode_inv(b)
        # print(' a = %s' % a)
        # print('a2 = %s' % a2)
        # print(' b = %s' % b)
        assert_allclose(a2[0], a)
