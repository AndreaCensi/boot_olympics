from contracts import contract

from bootstrapping_olympics import (UnsupportedSpec, RepresentationNuisance,
    NuisanceNotInvertible)
import numpy as np
from streamels import check_streamels_1D
from contracts.utils import check_isinstance


__all__ = ['Select']


class Select(RepresentationNuisance):
    ''' 
        A non-invertible nuisance that selects some of the sensels. 
        Works only for 1D streams.
    '''

    @contract(which='(int,>=0)|seq[>0](int,>=0)')
    def __init__(self, which):
        """
            Initializes this nuisance.
            
            :param which: Either an integer or a sequence of integers.
        """
        self.which = which

        if isinstance(which, int):
            self.select = np.array(range(which))
        else:
            self.select = np.array(which)

    def inverse(self):
        raise NuisanceNotInvertible()
    
    def left_inverse(self):
        raise NotImplementedError()

    def transform_streamels(self, streamels):
        check_streamels_1D(streamels)

        min_size = max(self.select) + 1
        if min_size > streamels.size:
            msg = ('This nuisance requires a stream with at least %d '
                   'streamels; found %s.' % (min_size, streamels.size))
            raise UnsupportedSpec(msg)

        streamels2 = streamels[self.select]

        self.expect_size = streamels.size

        return streamels2

    def transform_value(self, value):
        check_isinstance(value, np.ndarray)
        if value.size != self.expect_size:
            msg = 'Expected size %d, got shape %s.' % (self.expect_size, value.shape)
            raise ValueError(msg)
        return value[self.select]

    def __repr__(self):
        return 'Select(%s)' % self.which


