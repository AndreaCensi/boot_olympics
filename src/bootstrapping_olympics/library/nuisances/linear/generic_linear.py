from .. import (check_streamels_1D, check_streamels_continuous, contract, np,
    find_polytope_bounds_after_linear, check_streamels_1D_size)
from bootstrapping_olympics import (RepresentationNuisance,
    ValueFormats, streamel_dtype, NuisanceNotInvertible)

__all__ = ['GenericLinear']

# TODO: add left/right inverse, add kernel 
# TODO: preserve bounds


class GenericLinear(RepresentationNuisance):
    ''' 
        This represents a generic linear transformation
        between 1D continuous signals. The subclasses 
        only need to provide the matrix by either passing
        it in the constructor or implementing get_matrix().
    '''
    @contract(A='None|array[MxN]')
    def __init__(self, A=None):
        ''' A can be passed at construction time if it is known. '''
        # TODO: add kernel
        self.A0 = A
        self.A = None

    @contract(streamels='array', returns='array[MxN]')
    def get_matrix(self, streamels):
        ''' Returns the matrix representing the transformation. '''
        return None

    def _get_A(self, streamels):
        if self.A0 is not None:
            return self.A0
        else:
            A = self.get_matrix(streamels)
            if A is None:
                msg = ('A not passed to constructor and get_matrix not '
                       'implemented')
                raise ValueError(msg)
        return A

    def transform_streamels(self, streamels):
        check_streamels_1D(streamels)
        check_streamels_continuous(streamels)

        if self.A0 is not None:
            check_streamels_1D_size(streamels, self.A0.shape[1])

        self.A = self._get_A(streamels)
        if self.A.shape[1] != streamels.size:
            msg = ('I expect a matrix with rows %d, got %s' %
                   (streamels.size, self.A.shape))
            raise ValueError(msg)

        # TODO: use constr.
        streamels2 = np.zeros(self.A.shape[0], streamel_dtype)
        streamels2['kind'][:] = ValueFormats.Continuous
        find_polytope_bounds_after_linear(streamels, self.A, streamels2)
        self.lower = streamels2['lower'].copy()
        self.upper = streamels2['upper'].copy()

        streamels2['default'] = self.transform_value(streamels['default'])

        return streamels2

    def transform_value(self, value):
        value2 = np.dot(self.A, value)
        # enforce upper/lower bounds, to account for numerical errors
        value2 = value2.astype('float32')
        value2 = np.clip(value2, self.lower, self.upper)
        return value2

    def inverse(self):
        # TODO: left_inverse()
        M, N = self.A.shape
        if M != N:
            msg = 'Cannot invert %s matrix.' % str(self.A.shape)
            raise NuisanceNotInvertible(msg)

        # TODO: check for numerical errors
        Ainv = np.linalg.inv(self.A)

        return GenericLinear(Ainv)

    def __str__(self):
        if self.A is None:
            s = '<unset>'
        else:
            s = '(%dx%d)' % self.A.shape
        return 'GenericLinear(%s)' % s

