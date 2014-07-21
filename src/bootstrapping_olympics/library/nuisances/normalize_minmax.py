from bootstrapping_olympics import RepresentationNuisance
from contracts import contract
from contracts.utils import check_isinstance
from streamels import (ValueFormats, check_streamels_1D, 
    check_streamels_continuous, streamel_dtype)
import numpy as np
from streamels.base import BOOT_OLYMPICS_SENSEL_RESOLUTION


__all__ = ['NormalizeMinMax']


class NormalizeMinMax(RepresentationNuisance):
    ''' 
        Transforms: ::
        
            z[:n] = (y-mean(y)) /  max(y) - min(y)  \in -1,1
            z[n]  = mean(y)
            z[n+1]= max(y) - min(y)
    '''
    def transform_streamels(self, streamels):
        check_streamels_1D(streamels)
        check_streamels_continuous(streamels)

        n = streamels.size
        streamels2 = np.zeros(n + 2, streamel_dtype)
        streamels2['kind'][:] = ValueFormats.Continuous
        streamels2['lower'][:n] = -1
        streamels2['upper'][:n] = +1

        # this is the mean
        streamels2['lower'][n] = np.mean(streamels['lower'])
        streamels2['upper'][n] = np.mean(streamels['upper'])

        # this is the spread of the data        
        streamels2['lower'][n + 1] = 0
        # XXX: we should find a different bounded  
        streamels2['upper'][n + 1] = (np.max(streamels['upper']) - 
                                      np.min(streamels['lower']))

        # Save this so we can enforce it later
        self.lower = streamels2['lower'].copy()
        self.upper = streamels2['upper'].copy()
        self.default = streamels2['default'].copy()

        streamels2['default'] = self.transform_value(streamels['default'])

        return streamels2

    def inverse(self):
        return NormalizeMinMaxInverse(lower=self.lower, upper=self.upper, 
                                      default=self.default)

    def left_inverse(self):
        return self.inverse()

    def transform_value(self, value):
        check_isinstance(value, np.ndarray)
        mean = np.mean(value)
        spread = np.max(value) - np.min(value)

        n = value.size
        value2 = np.zeros(n + 2)
        if spread > 0:
            normalized = (value - mean) / spread
        else:
            normalized = (value - mean)
        value2[:n] = normalized
        value2[n] = mean
        value2[n + 1] = spread

        # enforce upper/lower bounds, to account for numerical errors
        return np.clip(value2.astype(BOOT_OLYMPICS_SENSEL_RESOLUTION), 
                       self.lower, self.upper)

    def __repr__(self):
        return 'NormalizeMinMax()'

 

class NormalizeMinMaxInverse(RepresentationNuisance):
    ''' 
        NormalizeMinMaxInverse transforms: ::
        
            z[:n] = (y-mean(y)) /  max(y) - min(y)  \in -1,1
            z[n]  = mean(y)
            z[n+1]= max(y) - min(y)
            
        The inverse is:
        
            mean = z[n]
            amp = z[n+1]
            y[:n] = z[:n] * amp + mean
    '''
        
    @contract(lower='array[N]',upper='array[N]',default='array[N]')
    def __init__(self, lower, upper, default):
        self.lower = lower
        self.upper = upper
        self.default = default
        
    def inverse(self):
        # XXX:
        return NormalizeMinMax()
    
    def left_inverse(self):
        return self.inverse()
 
    def transform_streamels(self, streamels):
        check_streamels_1D(streamels)
        check_streamels_continuous(streamels)

        nz = streamels.size
        n = nz - 2
        assert n == self.lower.size
        streamels2 = np.zeros(n, streamel_dtype)
        streamels2['kind'][:] = ValueFormats.Continuous
        streamels2['lower'][:] = self.lower
        streamels2['upper'][:] = self.upper
        streamels2['default'][:] = self.default
        return streamels2
    
    def transform_value(self, value):
        n = self.lower.size
        assert value.size == n
        amp = value[-1]
        mean = value[-2]
        z  = value[:n]
        y = z * amp + mean
        return y
    
