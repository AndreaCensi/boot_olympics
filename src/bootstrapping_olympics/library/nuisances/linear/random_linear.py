from .generic_linear import GenericLinear
from  contracts import contract
import numpy as np 


__all__ = ['RandomLinear']


class RandomLinear(GenericLinear):
    ''' A random linear transformation of the signal. '''

    @contract(seed='int')
    def __init__(self, seed, scale=1):
        self.seed = seed
        self.scale = scale
        GenericLinear.__init__(self)

    @contract(streamels='array[N]', returns='array[NxN]')
    def get_matrix(self, streamels):
        n = streamels.size
        # TODO: use our random generator
        np.random.seed(self.seed)
        A = np.random.randn(n, n) * self.scale
        return A

    def __str__(self):
        return 'RandomScaling(%s)' % (self.seed)


