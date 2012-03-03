from . import check_streamels_continuous, contract, np
from bootstrapping_olympics import (RepresentationNuisance, ValueFormats,
    NuisanceNotInvertible)

__all__ = ['Discretize']


class Discretize(RepresentationNuisance):
    ''' 
        This nuisance discretizes a float stream into 
        an integer representation.  
        
        There is an inverse, but it is imprecise. 
    '''

    @contract(levels='int,>=2')
    def __init__(self, levels):
        self.levels = levels

    def transform_streamels(self, streamels):
        check_streamels_continuous(streamels)

        # Save these so we can use them later
        self.lower = streamels['lower'].copy()
        self.upper = streamels['upper'].copy()

        streamels2 = np.empty_like(streamels)
        streamels2['kind'] = ValueFormats.Discrete
        streamels2['lower'] = 0
        streamels2['upper'] = self.levels - 1
        streamels2['default'] = self.transform_value(streamels['default'])
        return streamels2

    def transform_value(self, value):
        # Value normalized in [0,1]
        v01 = (value - self.lower) / (self.upper - self.lower)
        v01 = np.clip(v01, 0, +1)
        value2 = np.round(v01 * (self.levels - 1)).astype('int16')
        return value2

    def inverse(self):
        # TODO: write this
        raise NuisanceNotInvertible()

    def __repr__(self):
        return 'Discretize(%s)' % self.levels




