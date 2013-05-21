from bootstrapping_olympics import (NuisanceNotInvertible,
    make_streamels_2D_float, RepresentationNuisance, check_streamels_2D,
    check_streamels_continuous)
from contracts import contract
import numpy as np
import warnings


__all__ = ['Resample']


class Resample(RepresentationNuisance):
    ''' Resamples a 2D stream. Not invertible. '''

    @contract(shape='seq[2](int,>=1)')
    def __init__(self, shape):
        self.shape_to = shape

    def inverse(self):
        raise NuisanceNotInvertible()

    def transform_streamels(self, streamels):
        # Check that it is a 2D float image
        check_streamels_2D(streamels)
        check_streamels_continuous(streamels)
        self.shape_from = streamels.shape
        self.vmin = np.min(streamels['lower'])
        self.vmax = np.max(streamels['upper'])
        return make_streamels_2D_float(self.shape_to,
                                       lower=self.vmin, upper=self.vmax)

    @contract(value='array[HxW]')
    def transform_value(self, value):
        assert value.shape == self.shape_from
        from scipy.misc import imresize 
        y = imresize(value, self.shape_to, mode='F')
        y = np.array(y, dtype='float32')      
        warnings.warn('Hardcoded float32')
        assert y.min() >= self.vmin
        assert y.max() <= self.vmax
        return y

    def __repr__(self):
        return 'Resample({})'.format(self.shape_to)


