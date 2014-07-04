from blocks.composition import BBBBSeries
import numpy as np

from .instantaneous import Instantaneous
from .last_n_samples import LastNSamples


__all__ = ['SampledDeriv']


class SampledDeriv(BBBBSeries):
    def __init__(self):
        BBBBSeries.__init__(self, LastNSamples(3), ForwardDiff(), 'last3', 'fdiff')

    
class ForwardDiff(Instantaneous):
    """ 
        Implements 1-tap derivative.
        
        Assumes that the input is a list of 3 elements. 
    """

    def transform_value(self, value):
        assert len(value) == 3
        x1, x2, x3 = value
        assert len(x1) == 2
        assert len(x2) == 2
        assert len(x3) == 2
        t1, v1 = x1
        t2, v2 = x2
        t3, v3 = x3

        v1 = np.asarray(v1)
        v2 = np.asarray(v2)
        v3 = np.asarray(v3)

        # self.info('x1: %s' % str(x1))
        # self.info('x2: %s' % str(x2))
        # self.info('x3: %s' % str(x3))

        delta = t3 - t1
        if delta <= 0:
            msg = 'Invalid delta %r.' % delta
            raise ValueError(msg)
        
        if v1.dtype == np.dtype('uint8'):
            diff = v3.astype('float32') - v1.astype('float32')
        else:
            diff = v3 - v1

        timestamp = t2
        x = v2
        x_dot = diff / np.float32(delta)
        
        return (timestamp, (x, x_dot))
        


