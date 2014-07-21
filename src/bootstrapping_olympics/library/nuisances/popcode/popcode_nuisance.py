from bootstrapping_olympics import (RepresentationNuisance)
import numpy as np
from streamels import (streamel_dtype,
    ValueFormats, UnsupportedSpec, check_streamels_1D, check_streamels_continuous,
    check_streamels_2D, check_streamels_range)

from .popcode_functions import popcode, popcode_inv


__all__ = ['PopCode', 'PopCodeInv']


class PopCode(RepresentationNuisance):
    ''' 
        This nuisance transforms a 1D representation of the signal
        into a 2D representation. 
    '''

    def __init__(self, resolution=128):
        self.resolution = resolution

    def inverse(self):
        return PopCodeInv()
    
    def left_inverse(self):
        return self.inverse()
    
    def transform_streamels(self, streamels):
        check_streamels_1D(streamels)
        check_streamels_continuous(streamels)

        if not np.all(streamels['lower'] == 0):
            raise UnsupportedSpec("I only accept signals with lower=0.")

        if not np.all(streamels['upper'] == 1):
            raise UnsupportedSpec("I only accept signals with upper=1.")

        # TODO: check all floats and in 0--1

        shape2 = streamels.shape[0], self.resolution
        streamels2 = np.empty(shape=shape2, dtype=streamel_dtype)
        streamels2['kind'].flat[:] = ValueFormats.Continuous
        streamels2['lower'].flat[:] = 0
        streamels2['upper'].flat[:] = 1
        streamels2['default'] = popcode(streamels['default'], self.resolution)

        return streamels2

    def transform_value(self, values):
        return popcode(values, self.resolution)

    def __repr__(self):
        return 'PopCode(resolution=%d)' % self.resolution


class PopCodeInv(RepresentationNuisance):
    """ Inverse of PopCode """

    def inverse(self):
        return PopCode()  # XXX: cannot pass resolution
    
    def left_inverse(self):
        return PopCode()

    def transform_streamels(self, streamels):
        check_streamels_2D(streamels)
        check_streamels_continuous(streamels)
        check_streamels_range(streamels, 0, 1)

        N, _ = streamels.shape
        shape2 = N,
        streamels2 = np.empty(shape=shape2, dtype=streamel_dtype)
        streamels2['kind'].flat[:] = ValueFormats.Continuous
        streamels2['lower'].flat[:] = 0
        streamels2['upper'].flat[:] = 1
        streamels2['default'] = self.transform_value(streamels['default'])
        return streamels2

    def transform_value(self, values):
        return popcode_inv(values.astype('float32'))

    def __repr__(self):
        return 'PopCodeInv()'
