from bootstrapping_olympics import (NuisanceNotInvertible, RepresentationNuisance)
    
from streamels import (ValueFormats, check_streamels_continuous, streamel_dtype)
import numpy as np


__all__ = ['Normalize']


class Normalize(RepresentationNuisance):
    ''' 
        Transforms: ::
        
            z[:n] = (y-mean(y)) /  max(y) - min(y)  \in 0,1
            
        Look for NormalizeMinMax for a more advanced refinement of the 
        idea.
        
    '''

    def __init__(self, ymin=0, ymax=1):
        self.ymin = ymin
        self.ymax = ymax

    def inverse(self):
        raise NotImplementedError()

    def left_inverse(self):
        raise NotImplementedError()
    
    def transform_streamels(self, streamels):
        # check_streamels_1D(streamels)
        check_streamels_continuous(streamels)

        streamels2 = np.zeros(streamels.shape, streamel_dtype)
        streamels2['kind'][:] = ValueFormats.Continuous
        streamels2['lower'][:] = self.ymin
        streamels2['upper'][:] = self.ymax

        # Save this so we can enforce it later
        self.lower = streamels2['lower'].copy()
        self.upper = streamels2['upper'].copy()

        streamels2['default'] = self.transform_value(streamels['default'])

        return streamels2

    def transform_value(self, value):
        vmin = np.min(value)
        vmean = np.mean(value)
        vmax = np.max(value)
        spread = vmax - vmin

        
        value2 = (value - vmean) 
        if spread > 0:
            value2 = value2 / spread 
            
        value2 = value2 / 2 + 0.5
#         if spread > 0:
#             value2 = value2 / spread

#         print('min: %s max: %s' % (vmin, vmax))

        # enforce upper/lower bounds, to account for numerical errors
#         value2 = np.clip(value2, self.lower, self.upper)
        value2 = value2.astype('float32')
        
#        print('vmin: %s max: %s mean: %s' % (np.min(value2), np.max(value2), np.mean(value2)))
        return value2

    def __repr__(self):
        return 'Normalize()'

