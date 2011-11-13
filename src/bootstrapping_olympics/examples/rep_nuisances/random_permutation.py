from . import contract, np
from ...interfaces import StreamSpec, RepresentationNuisance, UnsupportedSpec

__all__ = ['RandomPermutation']

class RandomPermutation(RepresentationNuisance):
    ''' A fixed random permutation of the signal. '''
    
    @contract(seed='int', inverted='bool')
    def __init__(self, seed, inverted=False):
        self.seed = seed
        self.inverted = inverted
        self.perm = None
        
    def inverse(self):
        return RandomPermutation(self.seed, not self.inverted)
    
    @contract(stream_spec=StreamSpec)
    def transform_spec(self, stream_spec):
        # XXX: only allow 1D
        if 1 != len(stream_spec.shape()):
            raise UnsupportedSpec('Permutation only works with 1D signals.')
        
        n = stream_spec.size()
        self.perm = random_permutation(n, self.seed)
        if self.inverted:
            self.perm = invert_permutation(self.perm)
              
        streamels = stream_spec.get_streamels().copy()
        
        streamels['kind'] = streamels['kind'][self.perm]
        streamels['lower'] = streamels['lower'][self.perm]
        streamels['upper'] = streamels['upper'][self.perm]
        streamels['default'] = streamels['default'][self.perm]
        
        if stream_spec.id_stream:
            id_stream = stream_spec.id_stream + '-rperm%s' % self.seed
        else:
            id_stream = None
        return StreamSpec(id_stream=id_stream,
                          streamels=streamels,
                          extra=stream_spec.extra)
    
    def transform_value(self, values):
        if self.perm is None:
            raise ValueError('Please call transform_spec() first.')

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

