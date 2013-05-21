from .generic_linear import GenericLinear
import numpy as np 

__all__ = ['NormalizeMean']


class NormalizeMean(GenericLinear):
    ''' 
        This instantaneous transformation takes a vector of N values
        and transforms them into a vector of N+1 values.
        
        z[0:n-1] = y - mean(y)
        z[n] = mean(y) 
    '''

    def get_matrix(self, streamels):
        n = streamels.size
        A = np.zeros((n + 1, n))
        # z[0:n - 1] = y - mean(y)
        mean = np.ones(n) * 1.0 / n
        A[0:n, :] = np.eye(n)
        for i in range(n):
            A[i, :] -= mean
        # z[n] = mean(y)
        A[n, :] = mean
        return A

    def __repr__(self):
        return 'NormalizeMean()'


