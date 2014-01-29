from contracts import contract

import numpy as np

from .base import ValueFormats
from .stream_spec import streamels_all_of_kind


__all__ = [
    'check_streamels_1D',
    'check_streamels_2D',
    'check_streamels_1D_size',
    'check_streamels_continuous',
    'check_streamels_range',
    'check_1d_bit_sequence',
    'check_streamels_rgb'
]

@contract(streamels='streamel_array')
def check_streamels_1D(streamels):
    from bootstrapping_olympics import UnsupportedSpec
    if len(streamels.shape) != 1:
        msg = ('Only 1D streams are supported; shape is %s.'
                % str(streamels.shape))
        raise UnsupportedSpec(msg)


@contract(streamels='streamel_array')
def check_streamels_rgb(streamels):
    """ Checks that the streamels are RGB images (float between 0-1). """
    check_streamels_continuous(streamels)
    check_streamels_range(streamels, 0, 1)
    from bootstrapping_olympics import UnsupportedSpec
    def bail(m):
        msg = '%s\nstreamels: %s' % (m, streamels)
        raise UnsupportedSpec(msg)
    if len(streamels.shape) != 3:
        bail('Shape is not 3-dim.')
    if streamels.shape[2] != 3:
        bail('Shape does not have 3 components.')


@contract(streamels='streamel_array')
def check_streamels_2D(streamels):
    from bootstrapping_olympics import UnsupportedSpec
    if len(streamels.shape) != 2:
        msg = ('Only 2D streams are supported; shape is %s.'
                % str(streamels.shape))
        raise UnsupportedSpec(msg)


@contract(streamels='streamel_array', size='int,>0')
def check_streamels_1D_size(streamels, size):
    from bootstrapping_olympics import UnsupportedSpec
    check_streamels_1D(streamels)
    if streamels.size != size:
        msg = 'Expected size %d, obtained %d.' % (size, streamels.size)
        raise UnsupportedSpec(msg)



@contract(streamels='streamel_array')
def check_streamels_continuous(streamels):
    from bootstrapping_olympics import UnsupportedSpec
    '''
        Checks that all streamels have continuous data type. 
    
        :raise: UnsupportedSpec
    '''
    if not streamels_all_of_kind(streamels, ValueFormats.Continuous):
        msg = 'Only continuous streams supported.'
        raise UnsupportedSpec(msg)


@contract(streamels='streamel_array', lower='x', upper='>x')
def check_streamels_range(streamels, lower, upper):
    ''' 
        Checks that all streamels have the same range. 
    
        :raise: UnsupportedSpec
    '''
    from bootstrapping_olympics import UnsupportedSpec
    if not np.all(lower == streamels['lower']):
        msg = 'Not all streamels have lower bound %f.' % lower
        raise UnsupportedSpec(msg)
    if not np.all(upper == streamels['upper']):
        msg = 'Not all streamels have upper bound %f.' % lower
        raise UnsupportedSpec(msg)


def check_1d_bit_sequence(streamels, who):
    ''' Checks that it is a 1D sequence of bits (integers in {0,1}). '''
    from bootstrapping_olympics import UnsupportedSpec
    if not streamels_all_of_kind(streamels, ValueFormats.Discrete):
        msg = '%s only supports discrete streams.' % who
        raise UnsupportedSpec(msg)

    if streamels.ndim != 1:
        msg = '%s only works with 1D streams.' % who
        raise UnsupportedSpec(msg)

    if (np.any(streamels['lower'] != 0) or
         np.any(streamels['upper'] != 1)):
        msg = '%s expects only bits as input.' % who
        raise UnsupportedSpec(msg)
