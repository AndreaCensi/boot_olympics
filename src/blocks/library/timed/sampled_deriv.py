from blocks import SimpleBlackBoxT
from blocks.composition import BBBBSeriesT
from blocks.library.simple import InfoT, Instantaneous, LastNSamplesT
from contracts import contract
import numpy as np

__all__ = [
    'SampledDeriv',
    'SampledDerivInst',
]


def SampledDeriv():
    """ Warning: not instantaneous --- see SampledDerivPartial. """
    from blocks.composition import series
    S = series(InfoT('sampledinfo'), LastNSamplesT(3, send_partial=False), ForwardDiff(partial=False),
               InfoT('afterfdiff'))
    S.set_names(['info', 'last3', 'fdiff', 'afterfdiff'])
    return S

def SampledDerivInst():
    from blocks.composition import series
    S = series(LastNSamplesT(2, send_partial=True), ForwardDiff(partial=True))
    return S
    

class SampledDerivPartial(BBBBSeriesT):
    """ This one gives one output for each timestamp. The first one is (x, 0). """
    def __init__(self):
        BBBBSeriesT.__init__(self, LastNSamplesT(3, send_partial=True),
                            ForwardDiff(partial=True), 'last3', 'fdiff')


    
class ForwardDiff(Instantaneous, SimpleBlackBoxT):
    """ 
        Implements 1-tap derivative.
        
        Assumes that the input is a list of 3 elements. 
    """
    
    @contract(partial=bool)
    def __init__(self, partial):
        self.partial = partial
        Instantaneous.__init__(self)
    
    @contract(value='list(tuple(float, float|array))')
    def transform_value(self, value):
        assert len(value) in [1,2,3]
        
        if len(value) < 3 and not self.partial:
            raise ValueError(value)
            
        for x in value:
            self._check_single(x)
            
        if len(value) == 1:
            return self.deriv1(value)
        if len(value) == 2:
            return self.deriv2(value)
        if len(value) == 3:
            return self.deriv3(value)
        
    @contract(tx='tuple(float, float)')
    def _check_single(self, tx):
        pass
        
    def deriv1(self, value):
        # TODO
        t, x = value[0]
        x_dot = x*0.0
        return (t, (x, x_dot))

    def deriv2(self, value):
        x1, x2 = value
        assert len(x1) == 2
        assert len(x2) == 2
        t1, v1 = x1
        t2, v2 = x2
        
        if not t1 < t2: 
            msg = 'Invalid timestamps t1 %s t2 %s  ' % (t1, t2)
            raise ValueError(msg)
 
        x_dot = self.make_difference(t1, v1, t2, v2)

        timestamp = t2
        x = v2 
        return (timestamp, (x, x_dot))
            
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
 

        x_dot = self.make_difference(t1, v1, t3, v3)

        timestamp = t2
        x = v2 
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


