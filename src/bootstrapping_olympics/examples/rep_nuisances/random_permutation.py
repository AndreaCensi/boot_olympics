from . import contract, np
from ...interfaces import StreamSpec, RepresentationNuisance

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
        
        n = stream_spec.size()
        self.perm = random_permutation(n, self.seed)
        if self.inverted:
            self.perm = invert_permutation(self.perm)
            
        # Doesn't change the spec
        # FIXME: spec may change!
        return stream_spec
    
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
            data = ',%s' % self.scale
        return 'Perm(%s,%s%s)' % (self.seed, self.inverted, data)


def random_permutation(n, seed):
    ''' Returns a random permutation of n elements. '''
    # TODO: do not touch the global seed
    np.random.seed(seed)
    return np.argsort(np.random.rand(n))

def invert_permutation(perm):
    ''' Inverts a permutation. '''
    return np.argsort(perm)

