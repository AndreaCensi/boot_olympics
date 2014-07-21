from contracts import contract

from bootstrapping_olympics import (RepresentationNuisance)
import numpy as np
from streamels import new_streamels, check_streamels_1D
from contracts.utils import check_isinstance


__all__ = [
    'RandomPermutation',
]


class RandomPermutation(RepresentationNuisance):
    ''' A fixed random permutation of the signal. '''

    @contract(seed='int', inverted='bool')
    def __init__(self, seed, inverted=False):
        self.seed = seed
        self.inverted = inverted
        self.perm = None

    def inverse(self):
        return RandomPermutation(self.seed, not self.inverted)

    def left_inverse(self):
        return self.inverse()

    def transform_streamels(self, streamels):
        check_streamels_1D(streamels)

        n = streamels.size
        self.perm = random_permutation(n, self.seed)
        if self.inverted:
            self.perm = invert_permutation(self.perm)

        streamels2 = new_streamels(streamels.shape)
        streamels2['kind'] = streamels['kind'][self.perm]
        streamels2['lower'] = streamels['lower'][self.perm]
        streamels2['upper'] = streamels['upper'][self.perm]
        streamels2['default'] = self.transform_value(streamels['default'])
        return streamels2

    def transform_value(self, values):
        check_isinstance(values, np.ndarray)
        if self.perm is None:
            msg = 'Please call transform_spec() first.'
            self.debug(msg)
            raise ValueError(msg)

        other = np.empty_like(values)
        other.flat[:] = values.flat[self.perm]
        return other

    def __str__(self):
        if self.perm is None:
            data = ""
        else:
            data = ',%s' % self.perm
        return 'Perm(%s,%s%s)' % (self.seed, self.inverted, data)


def random_permutation(n, seed):
    ''' Returns a random permutation of n elements. '''
    # TODO: do not touch the global seed
    np.random.seed(seed)
    return np.argsort(np.random.rand(n))


def invert_permutation(perm):
    ''' Inverts a permutation. '''
    return np.argsort(perm)

