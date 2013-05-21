
""" 
    A bunch of diffeomorphisms of the real line.
"""
import numpy as np
from contracts import contract
from .. import GenericScalar


@contract(x='array', alpha='>0')
def power(x, alpha=2.0):
    return np.sign(x) * np.power(np.abs(x), alpha)

#
# @contract(x='array', alpha='>0')
# def powerinv(x, alpha=2):
#    return power(x, 1.0 / alpha)


def to01(f):
    """ Maps the result of a function from [-1,1] to [0,1] """
    def wrap(x, *args, **kwargs):
        y = x * 2 - 1
        z = f(y, *args, **kwargs)
        return (z + 1) / 2
    return wrap
    
power01 = to01(power)
# power01inv = to01(powerinv)

#
# def nlf1(x):
#    return np.sin(np.pi * x / 2)


class Power(GenericScalar):
    
    @contract(alpha='>0')
    def __init__(self, alpha=2):
        self.alpha = alpha
    
    @contract(x='array')
    def transform01(self, x):
        return power01(x, self.alpha)
    
    def inverse(self):
        return Power(1.0 / self.alpha)
    
