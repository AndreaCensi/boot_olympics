from . import GenericLinear
from .. import contract, np


__all__ = ['RandomScaling']


class RandomScaling(GenericLinear):
    ''' A fixed random scaling of the signal. '''

    @contract(seed='int')
    def __init__(self, seed):
        self.seed = seed

        GenericLinear.__init__(self)

    @contract(streamels='array[N]', returns='array[NxN]')
    def get_matrix(self, streamels):
        n = streamels.size
        # TODO: use our random generator
        np.random.seed(self.seed)
        S = np.random.exponential(scale=1.0, size=n)
        #print('Found matrix: %s %s %s' % (self.seed, S.sum(), 0))
        return np.diag(S)

    def __str__(self):
        return 'RandomScaling(%s)' % (self.seed)


