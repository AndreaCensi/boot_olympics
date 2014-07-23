from bootstrapping_olympics import RepresentationNuisance
from contracts.utils import check_isinstance
from streamels import (BOOT_OLYMPICS_SENSEL_RESOLUTION, ValueFormats, 
    check_streamels_1D, check_streamels_continuous, streamel_dtype)
import numpy as np


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

        self.streamels_orig = streamels.copy()
        self.streamels2 = streamels2.copy()
        return streamels2

    def inverse(self):
        return NormalizeMinMaxInverse(streamels_orig=self.streamels_orig.copy(),
                                      streamels2=self.streamels2.copy())

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
        
#     @contract(lower='array[N]',upper='array[N]',default='array[N]')
    def __init__(self, streamels_orig, streamels2):
        self.streamels_orig = streamels_orig
        self.streamels2 = streamels2
        
    def inverse(self):
        # XXX:
        return NormalizeMinMax()
    
    def left_inverse(self):
        return self.inverse()
 
    def transform_streamels(self, streamels):
        assert streamels.shape == self.streamels2.shape # XXX: and everything else
        check_streamels_1D(streamels)
        check_streamels_continuous(streamels)
        return self.streamels_orig
    
    def transform_value(self, value):
        n = self.streamels_orig.size
        assert value.size == n + 2
        amp = value[-1]
        mean = value[-2]
        z  = value[:n]
        y = z * amp + mean
        return y
    
