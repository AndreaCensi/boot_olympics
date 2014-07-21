
from contracts import contract

from bootstrapping_olympics import RepresentationNuisance, NuisanceNotInvertible
import numpy as np
from streamels import (check_streamels_rgb,
    make_streamels_2D_float)
from streamels.streamels_make import make_streamels_rgb_float
from streamels.streamels_checks import check_streamels_2D,\
    check_streamels_continuous


__all__ = [
    'LumFromRGB', 
    'RGBFromLum',
]


class LumFromRGB(RepresentationNuisance):
    ''' 
        Converts an RGB signal (float shape HxWx3) into grayscale. 
        
        This is a non-invertible nuisance.
    '''

    def __init__(self):
        pass

    def inverse(self):
        raise NuisanceNotInvertible()
        
    def left_inverse(self):
        return RGBFromLum()

    def transform_streamels(self, streamels):
        check_streamels_rgb(streamels)
        H, W, c = streamels.shape
        assert c == 3
        return make_streamels_2D_float([H, W], 0, 1)

    def transform_value(self, values):
        return luminance_from_rgb_float(values)

    def __str__(self):
        return 'LumFromRGB()'



class RGBFromLum(RepresentationNuisance):
    ''' 
        Converts an RGB signal (float shape HxWx3) into grayscale. 
        
        This is a non-invertible nuisance.
    '''

    def __init__(self):
        pass

    def inverse(self):
        return RGBFromLum()
        
    def left_inverse(self):
        return self.inverse()

    def transform_streamels(self, streamels):
        check_streamels_continuous(streamels)
        check_streamels_2D(streamels)
        H, W = streamels.shape
        return make_streamels_rgb_float([H, W])

    def transform_value(self, values):
        return rgb_from_luminance_float(values)

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



@contract(returns='array[HxWx3](float)', gray='array[HxW](float)')
def rgb_from_luminance_float(gray):
    """ 
        Converts a HxWx3 RGB image into a HxW grayscale image 
        by computing the luminance. 
    """ 
    H, W = gray.shape
    rgb = np.zeros((H, W, 3), 'float')
    rgb[:,:,0] = gray
    rgb[:,:,1] = gray
    rgb[:,:,2] = gray
    return rgb


