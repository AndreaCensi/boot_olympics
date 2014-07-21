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
            
            :param which: Either an integer or a sequence of integers
                representing the indices of the elements to select.
        """

        if isinstance(which, int):
            self.select = np.array(range(which))
        else:
            self.select = np.array(which)

        self.which = list(self.select)


    def inverse(self):
        raise NuisanceNotInvertible()
    
    def left_inverse(self):
        return SelectLeftInverse(which=self.which, orig_streamels=self.orig_streamels)

    def transform_streamels(self, streamels):
        check_streamels_1D(streamels)

        self.orig_streamels = streamels.copy()
        
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




class SelectLeftInverse(RepresentationNuisance):
    
    @contract(which='(int,>=0)|seq[>0](int,>=0)')
    def __init__(self, orig_streamels, which):
        self.orig_streamels = orig_streamels
        self.which = which
        
    def inverse(self):
        return Select(self.which)
    
    def left_inverse(self):
        return self.inverse()
    
    def transform_streamels(self, streamels):
        assert streamels.size == len(self.which)
        return self.orig_streamels.copy()

    def transform_value(self, value):
        value2 = self.orig_streamels['default']
        for i, x in enumerate(self.which):
            value2[x] = value[i]
        return value2
        
        
        
        
        
