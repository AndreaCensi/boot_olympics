from .generic_linear import GenericLinear
from  contracts import contract
import numpy as np 


__all__ = ['RandomScaling']


class RandomScaling(GenericLinear):
    ''' A fixed random scaling of the signal. '''

    @contract(seed='int', scale='float,>0')
    def __init__(self, seed, scale=1.0):
        self.seed = seed
        self.scale = scale

        GenericLinear.__init__(self)

    @contract(streamels='array[N]', returns='array[NxN]')
    def get_matrix(self, streamels):
        n = streamels.size
        # TODO: use our random generator
        np.random.seed(self.seed)
        S = np.random.exponential(scale=self.scale, size=n)
        # print('Found matrix: %s %s %s' % (self.seed, S.sum(), 0))
        return np.diag(S)

    def __str__(self):
        return 'RandomScaling(%s)' % (self.seed)


