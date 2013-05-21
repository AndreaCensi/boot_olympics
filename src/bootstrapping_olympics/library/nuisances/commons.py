from numpy.core.numeric import allclose
from contracts import contract
import numpy as np
from bootstrapping_olympics import check_streamels_1D_size

 
__all__ = ['find_polytope_bounds_after_linear']

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

