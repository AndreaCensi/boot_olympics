from numpy.core.numeric import allclose
from contracts import contract
import numpy as np
from streamels import check_streamels_1D_size

 
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
        n, _ =A.shape
        for i in range(n):
            a = A[i,i]
            if a > 0:
                l = streamels['lower'][i] * a 
                u = streamels['upper'][i] * a
            else:
                u = streamels['lower'][i] * a
                l = streamels['upper'][i] * a
        
            assert l <= u
                
            streamels2['lower'][i] = l
            streamels2['upper'][i] = u


    else:  # TODO: do something smarter here
        norm = np.abs(A).sum()  # XXX
        lb = np.max(np.abs(streamels['lower']))
        ub = np.max(np.abs(streamels['upper']))
        B = max(lb, ub)
        streamels2['lower'] = -B * norm
        streamels2['upper'] = +B * norm
        
    
    l = streamels2['lower']
    u = streamels2['upper']
    assert np.all(l <= u)

