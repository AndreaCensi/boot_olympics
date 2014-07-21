from bootstrapping_olympics import NuisanceNotInvertible, RepresentationNuisance
from contracts import contract
from streamels import ValueFormats, check_streamels_continuous
import numpy as np
from contracts.utils import check_isinstance

__all__ = [
    'Discretize', 
    'DiscretizeLeftInverse',
]


class Discretize(RepresentationNuisance):
    ''' 
        This nuisance discretizes a float stream into 
        an integer representation.  
        
        There is only an approximated inverse. 
    '''

    @contract(levels='int,>=2')
    def __init__(self, levels):
        self.levels = levels

    def transform_streamels(self, streamels):
        check_streamels_continuous(streamels)

        # Save these so we can use them later
        self.lower = streamels['lower'].copy()
        self.upper = streamels['upper'].copy()
        self.default = streamels['default'].copy()

        streamels2 = np.empty_like(streamels)
        streamels2['kind'] = ValueFormats.Discrete
        streamels2['lower'] = 0
        streamels2['upper'] = self.levels - 1
        streamels2['default'] = self.transform_value(self.default)
        return streamels2

    @contract(value='array')
    def transform_value(self, value):
        check_isinstance(value, np.ndarray)
        assert value.shape == self.lower.shape, value
        # Value normalized in [0,1]
        v01 = (value - self.lower) / (self.upper - self.lower)
        v01 = np.clip(v01, 0, +1)
        value2 = np.round(v01 * (self.levels - 1)).astype('int16')
        return value2

    def inverse(self):
        raise NuisanceNotInvertible()

    def left_inverse(self):
        return DiscretizeLeftInverse(levels=self.levels, lower=self.lower,
                                upper=self.upper, default = self.default)

    def __repr__(self):
        return 'Discretize(%s)' % self.levels


class DiscretizeLeftInverse(RepresentationNuisance):
    
    def __init__(self, levels, upper, lower, default):
        self.levels = levels
        self.upper = upper
        self.lower = lower
        self.default = default

    def left_inverse(self):
        return Discretize(self.levels)
    
    def inverse(self):
        return self.left_inverse()
        
    def transform_streamels(self, streamels):
        if streamels.shape != self.lower.shape:
            msg = 'invalid shape'
            raise ValueError(msg)
        
        streamels2 = np.empty_like(streamels)
        streamels2['kind'] = ValueFormats.Continuous
        streamels2['lower'] = self.lower
        streamels2['upper'] = self.upper
        streamels2['default'] = self.default
        return streamels2
    
    def transform_value(self, value):
        # from 0....levels-1 to 0...1
        assert np.all(value>=0) and np.all(value<=self.levels-1)
        res01 = value * 1.0 / (self.levels-1)
        res = self.lower  + (self.upper - self.lower)* res01
        return res

    def __repr__(self):
        return 'DiscretizeLeftInverse(%s)' % self.levels


