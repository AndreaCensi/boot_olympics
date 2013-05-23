from bootstrapping_olympics import (NuisanceNotInvertible,
    make_streamels_2D_float, RepresentationNuisance, check_streamels_2D,
    check_streamels_continuous)
from contracts import contract
import numpy as np
import warnings
import scipy


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

@contract(image='array[HxWx3](float32)|array[HxWx3](uint8)|array[HxW](float32)',
          shape='seq[2](int,>1)', returns='array')
def scipy_image_resample(image, shape, order=3):
    """ 
        :param order: Order of spline interpolation. order=0 should be invertible
        if upsampling.
    """
#     if image.dtype == 'float32' and len(image.shape) == 3:
#         y1 = (image * 255).astype('uint8')
#         y1r = scipy_image_resample(y1, shape)
#         return y1r * (1.0 / 255.0)
#     
#     assert image.dtype == 'uint8', image.dtype
#     warnings.warn('extra depedency to remove')
#     from procgraph_pil import resize 
#     y = resize(image, width=shape[1], height=shape[0])
#     print('old shape: %s' % str(image.shape))
#     print('want shape: %s' % str(shape))
#     print('new shape: %s' % str(y.shape))
    import scipy.ndimage
    f = scipy.ndimage.interpolation.zoom
    
    shape_target = list(image.shape)
    shape_target[0] = shape[0]
    shape_target[1] = shape[1]
    shape_target = tuple(shape_target)
    # 3 = cubic
    zoom = [1.0 * shape[0] / image.shape[0],
            1.0 * shape[1] / image.shape[1]]
    if len(image.shape) == 3:
        zoom.append(1)
    # print('zoom: %s' % str(zoom))
    
    y = f(image, zoom=zoom, order=order, mode='nearest')
    # print('target: %s' % str(shape_target))
    # print('obtained: %s' % str(y.shape))
    
#     from scipy.misc import imresize
#     y = imresize(image, shape)  # , mode='F')
    assert y.shape[0] == shape[0]
    assert y.shape[1] == shape[1]
    assert len(y.shape) == len(image.shape)
    return y

    


