from . import contract, np
from bootstrapping_olympics import (UnsupportedSpec, streamels_all_of_kind,
    ValueFormats)
from numpy.core.numeric import allclose


@contract(streamels='streamel_array')
def check_streamels_1D(streamels):
    if len(streamels.shape) != 1:
        msg = ('Only 1D streams are supported; shape is %s.'
                % str(streamels.shape))
        raise UnsupportedSpec(msg)

@contract(streamels='streamel_array')
def check_streamels_2D(streamels):
    if len(streamels.shape) != 2:
        msg = ('Only 2D streams are supported; shape is %s.'
                % str(streamels.shape))
        raise UnsupportedSpec(msg)


@contract(streamels='streamel_array', size='int,>0')
def check_streamels_1D_size(streamels, size):
    check_streamels_1D(streamels)
    if streamels.size != size:
        msg = 'Expected size %d, obtained %d.' % (size, streamels.size)
        raise UnsupportedSpec(msg)


@contract(streamels='streamel_array')
def check_streamels_continuous(streamels):
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
    if not np.all(lower == streamels['lower']):
        msg = 'Not all streamels have lower bound %f.' % lower
        raise UnsupportedSpec(msg)
    if not np.all(upper == streamels['upper']):
        msg = 'Not all streamels have upper bound %f.' % lower
        raise UnsupportedSpec(msg)


def check_1d_bit_sequence(streamels, who):
    ''' Checks that it is a 1D sequence of bits (integers in {0,1}). '''
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


@contract(streamels='streamel_array', streamels2='streamel_array',
          A='array[MxN]')
def find_polytope_bounds_after_linear(streamels, A, streamels2):
    ''' 
        Fills the 'lower' and 'upper' part of streamels2
        given the A transform.
    '''
    M, N = A.shape
    check_streamels_1D_size(streamels, N)
    check_streamels_1D_size(streamels2, M)

    is_diagonal = (M == N) and allclose(np.diag(np.diagonal(A)), A)

    if is_diagonal:
        streamels2['lower'] = np.dot(A, streamels['lower'])
        streamels2['upper'] = np.dot(A, streamels['upper'])

    else:  # TODO: do something smarter here
        norm = np.abs(A).sum()  # XXX
        lb = np.max(np.abs(streamels['lower']))
        ub = np.max(np.abs(streamels['upper']))
        B = max(lb, ub)
        streamels2['lower'] = -B * norm
        streamels2['upper'] = +B * norm

