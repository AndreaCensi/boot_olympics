from bootstrapping_olympics import (NuisanceNotInvertible,
    make_streamels_2D_float, RepresentationNuisance, check_streamels_2D,
    check_streamels_continuous)
from contracts import contract
import numpy as np
import warnings


__all__ = ['Resample', 'scipy_image_resample']


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

@contract(image='array[HxWx3](float32)|array', shape='seq[2](int,>1), x', returns='shape(x)')
def scipy_image_resample(image, shape):
    if image.dtype == 'float32' and len(image.shape) == 3:
        y1 = (image * 255).astype('uint8')
        y1r = scipy_image_resample(y1, shape)
        return y1r / 255.0
    warnings.warn('extra depedency to remove')
    from procgraph_pil import resize 
    y = resize(image, width=shape[1], height=shape[0])
#     from scipy.misc import imresize
#     y = imresize(image, shape, mode='F')
    return y

    


