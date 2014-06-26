
from contracts import contract

from bootstrapping_olympics import RepresentationNuisance, NuisanceNotInvertible
import numpy as np
from streamels import (check_streamels_rgb,
    make_streamels_2D_float)


__all__ = ['LumFromRGB']


class LumFromRGB(RepresentationNuisance):
    ''' 
        Converts an RGB signal (float shape HxWx3) into grayscale. 
        
        This is a non-invertible nuisance.
    '''

    def __init__(self):
        pass

    def inverse(self):
        raise NuisanceNotInvertible()

    def transform_streamels(self, streamels):
        check_streamels_rgb(streamels)
        H, W, c = streamels.shape
        assert c == 3
        return make_streamels_2D_float([H, W], 0, 1)

    def transform_value(self, values):
        return luminance_from_rgb_float(values)

    def __str__(self):
        return 'LumFromRGB()'


@contract(rgb='array[HxWx3](float)', returns='array[HxW](float)')
def luminance_from_rgb_float(rgb):
    """ 
        Converts a HxWx3 RGB image into a HxW grayscale image 
        by computing the luminance. 
    """ 
    r = rgb[:, :, 0].squeeze()
    g = rgb[:, :, 1].squeeze()
    b = rgb[:, :, 2].squeeze()
    # note we keep a uint8
    gray = r * 299.0 / 1000 + g * 587.0 / 1000 + b * 114.0 / 1000
    gray = np.clip(gray, 0, 1)
    return gray

