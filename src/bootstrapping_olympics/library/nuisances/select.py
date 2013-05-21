from bootstrapping_olympics import (UnsupportedSpec, RepresentationNuisance,
    NuisanceNotInvertible, check_streamels_1D)
from contracts import contract
import numpy as np

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

    def transform_streamels(self, streamels):
        check_streamels_1D(streamels)

        min_size = max(self.select) + 1
        if min_size > streamels.size:
            msg = ('This nuisance requires a stream with at least %d '
                   'streamels; found %s.' % (min_size, streamels.size))
            raise UnsupportedSpec(msg)

        streamels2 = streamels[self.select]
        return streamels2

    def transform_value(self, value):
        return value[self.select]

    def __repr__(self):
        return 'Select(%s)' % self.which


