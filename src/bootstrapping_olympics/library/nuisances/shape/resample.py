from bootstrapping_olympics import (NuisanceNotInvertible,
    make_streamels_2D_float, RepresentationNuisance, check_streamels_2D,
    check_streamels_continuous)
from contracts import contract
import numpy as np
import warnings
from numpy.testing.utils import assert_allclose


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
    import scipy.ndimage
    
    shape_target = list(image.shape)
    shape_target[0] = shape[0]
    shape_target[1] = shape[1]
    shape_target = tuple(shape_target)
    # 3 = cubic

    if True:
        eps = 0.0001
        zoom = [1.0 * (shape[0] + eps) / image.shape[0],
                1.0 * (shape[1] + eps) / image.shape[1]]
    
        if len(image.shape) == 3:
            zoom.append(1)

        # this is the way it is computed by scipy, using "int"
        projected = tuple([int(ii * jj) for ii, jj in zip(image.shape, zoom)])
        assert_allclose(projected, shape_target)
            
        # print('image orig: %s' % str(image.shape))
        # print('want: %s' % str(shape))
        # print('zoom: %s' % str(zoom))
        # s = np.array(zoom) * np.array(image.shape)
        # print('image orig * zoom: %s' % str(s))
        f = scipy.ndimage.interpolation.zoom
        y = f(image, zoom=zoom, order=order, mode='nearest')

#     else:
#         y = scipy_zoom(image, shape_target, order=order, mode='nearest')
    # print('target: %s' % str(shape_target))
    # print('obtained: %s' % str(y.shape))
    
#     from scipy.misc import imresize
#     y = imresize(image, shape)  # , mode='F')
    assert_allclose(y.shape, shape_target)
#     assert y.shape[0] == shape[0]
#     assert y.shape[1] == shape[1]
#     assert len(y.shape) == len(image.shape)
    return y
# 
# 
# def scipy_zoom(input, output_shape, output=None, order=3, mode='constant', cval=0.0):
#     """
#    
#     """
#     import numpy 
#     from scipy.ndimage import _ni_support, _nd_image
#     filtered = input
#     
#     zoom_div = numpy.array(output_shape, float) - 1
#     zoom = (numpy.array(input.shape) - 1) / zoom_div
# 
#     # Zooming to infinity is unpredictable, so just choose
#     # zoom factor 1 instead
#     zoom[numpy.isinf(zoom)] = 1
# 
#     output, return_value = _ni_support._get_output(output, input,
#                                                    shape=output_shape)
#     zoom = numpy.asarray(zoom, dtype=numpy.float64)
#     zoom = numpy.ascontiguousarray(zoom)
# 
#     
#     print('zoom: %r ' % zoom)
#     print(zoom, None, output, order, mode, cval)
#     _nd_image.zoom_shift(filtered, zoom, None, output, order, mode, cval)
#     return return_value

