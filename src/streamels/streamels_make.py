from contracts import contract

import numpy as np

from .base import streamel_dtype, ValueFormats


__all__ = [
    'new_streamels',
    'make_streamels_2D_float',
    'make_streamel_bit',
    'streamels_join_1D',
    'make_streamels_rgb_float',
    'make_streamels_float',
    'make_streamels_1D_float',
    'make_streamels_finite_commands',
]

@contract(shape='(int,>0)|seq[>0](int,>0)')
def new_streamels(shape):
    ''' 
        Creates a new array of streamels, initialized with all invalid
        values. This is useful to check if we forgot to initialize 
        some fields.
    '''
    x = np.zeros(shape, streamel_dtype)
    x['kind'] = '?'
    x['lower'] = np.nan
    x['upper'] = np.nan
    x['default'] = np.nan
    return x


@contract(shape='seq[2](int,>=1)', lower='number,finite,x',
          upper='number,finite,y,>x', vdef='None|(number,finite,>x,<y)')
def make_streamels_2D_float(shape, lower, upper, vdef=None):
    """ Creates a stremeals spec with the given shape, min and max. """
    x = np.zeros(shape, streamel_dtype)
    x['kind'] = ValueFormats.Continuous
    x['lower'] = lower
    x['upper'] = upper
    if vdef is None: 
        vdef = lower * 0.5 + upper * 0.5
    x['default'] = vdef
    return x


@contract(shape='seq[2](int)')
def make_streamels_rgb_float(shape):
    """ An image, using floats instead of uint8. """
    x = np.zeros((shape[0], shape[1], 3), streamel_dtype)
    x['kind'] = ValueFormats.Continuous
    x['lower'] = 0
    x['upper'] = 1
    x['default'] = 0.5
    return x


@contract(shape='seq[>=1](int,>=1)', lower='number,finite,x',
          upper='number,finite,y,>x', vdef='None|(number,finite,>x,<y)')
def make_streamels_float(shape, lower, upper, vdef=None):
    """ Creates a stremeals spec with the given shape, min and max. """
    x = np.zeros(shape, streamel_dtype)
    x['kind'] = ValueFormats.Continuous
    x['lower'] = lower
    x['upper'] = upper
    if vdef is None: 
        vdef = lower * 0.5 + upper * 0.5
    x['default'] = vdef
    return x

    
@contract(nstreamels='int,>=1',
          lower='number,finite,x',
          upper='number,finite,y,>x',
          vdef='None|(number,finite,>x,<y)')
def make_streamels_1D_float(nstreamels, lower, upper, vdef=None):
    x = np.zeros(nstreamels, streamel_dtype)
    x['kind'] = ValueFormats.Continuous
    x['lower'] = lower
    x['upper'] = upper
    if vdef is None: 
        vdef = lower * 0.5 + upper * 0.5
    x['default'] = vdef
    return x
    

def make_streamels_finite_commands(ncommands, default):
    """ Creates a spec with 1 streamel taking integer values
        representing different commands. """
    x = np.zeros(1, streamel_dtype)
    x['kind'] = ValueFormats.Discrete
    x['lower'] = 0
    x['upper'] = ncommands - 1
    x['default'] = default
    return x


def make_streamel_bit(default=0):
    """ Creates a spec with 1 streamel which takes values 0,1  """
    x = np.zeros(1, streamel_dtype)
    x['kind'] = ValueFormats.Discrete
    x['lower'] = 0
    x['upper'] = 1
    x['default'] = default
    return x


@contract(s1='array[N]', s2='array[M]', returns='array[M+N]')
def streamels_join_1D(s1, s2):
    return np.hstack((s1, s2))
    
    
    
