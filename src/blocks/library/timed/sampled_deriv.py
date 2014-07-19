from blocks.composition import BBBBSeries
from blocks.library.simple import Instantaneous, LastNSamplesT
import numpy as np


__all__ = ['SampledDeriv']


class SampledDeriv(BBBBSeries):
    """ Warning: not instantaneous --- see SampledDerivPartial. """
    def __init__(self):
        BBBBSeries.__init__(self, LastNSamplesT(3, send_partial=False),
                            ForwardDiff(partial=False), 'last3', 'fdiff')


class SampledDerivPartial(BBBBSeries):
    """ This one gives one output for each timestamp. The first one is (x, 0). """
    def __init__(self):
        BBBBSeries.__init__(self, LastNSamplesT(3, send_partial=True),
                            ForwardDiff(partial=True), 'last3', 'fdiff')


    
class ForwardDiff(Instantaneous):
    """ 
        Implements 1-tap derivative.
        
        Assumes that the input is a list of 3 elements. 
    """
    def __init__(self, partial):
        self.partial = partial
        Instantaneous.__init__(self)

    def transform_value(self, value):
        assert len(value) in [1,2,3]
        
        if len(value) < 3 and not self.partial:
            raise ValueError(value)
            
        if len(value) == 1:
            return self.deriv1(value)
        if len(value) == 2:
            return self.deriv2(value)
        if len(value) == 3:
            return self.deriv3(value)
        
    def deriv1(self, value):
        # TODO
        raise NotImplementedError()

    def deriv2(self, value):
#         assert len(value)
#         x1 = value[0]
#         t1, v1 = x1
#         v1 = np.asarray(v1)
#
#         # TODO
        raise NotImplementedError()
            
#         x_dot = self.make_difference()
            
    def deriv3(self, value):
        x1, x2, x3 = value
        assert len(x1) == 2
        assert len(x2) == 2
        assert len(x3) == 2
        t1, v1 = x1
        t2, v2 = x2
        t3, v3 = x3

        if not t1 < t2 < t3:
            msg = 'Invalid timestamps t1 %s t2 %s t3 %s' % (t1, t2, t3)
            raise ValueError(msg)

#         v1 = np.asarray(v1)
#         v2 = np.asarray(v2)
#         v3 = np.asarray(v3)

        # self.info('x1: %s' % str(x1))
        # self.info('x2: %s' % str(x2))
        # self.info('x3: %s' % str(x3))

        x_dot = self.make_difference(t1, v1, t3, v3)


#
#         delta = t3 - t1
#         if delta <= 0:
#             msg = 'Invalid delta %r.' % delta
#             raise ValueError(msg)
#
#         if v1.dtype == np.dtype('uint8'):
#             diff = v3.astype('float32') - v1.astype('float32')
#         else:
#             diff = v3 - v1

        timestamp = t2
        x = v2
#         x_dot = diff / np.float32(delta)
        
        return (timestamp, (x, x_dot))
        
    def make_difference(self, tA, vA, tB, vB):
        vA = np.asarray(vA)
        vB = np.asarray(vB)
        
        delta = tB - tA
        if delta <= 0:
            msg = 'Invalid delta %r (tA = %f tb = %f).' % (delta, tA, tB)
            raise ValueError(msg)

        if vA.dtype == np.dtype('uint8'):
            diff = vB.astype('float32') - vA.astype('float32')
        else:
            diff = vB - vA

        x_dot = diff / np.float32(delta)

        return x_dot


