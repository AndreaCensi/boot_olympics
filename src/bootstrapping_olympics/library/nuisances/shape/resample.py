from bootstrapping_olympics import (NuisanceNotInvertible, 
    NuisanceNotLeftInvertible, RepresentationNuisance)
from contracts import contract
from numpy.testing.utils import assert_allclose
from streamels import (check_streamels_2D, check_streamels_continuous, 
    make_streamels_2D_float)
import numpy as np
import warnings


__all__ = [
    'Resample', 
    'scipy_image_resample',
]


class Resample(RepresentationNuisance):
    ''' Resamples a 2D stream. Not invertible. '''

    @contract(shape='seq[2](int,>=1)')
    def __init__(self, shape, be_liberal=True):
        self.shape_to = shape
        self.be_liberal = be_liberal
        self.warned = False

    def inverse(self):
        raise NuisanceNotInvertible()

    def left_inverse(self):
        # we don't have a reversible implementation
        raise NuisanceNotLeftInvertible()
        
    def left_inverse_approx(self):
        return Resample(self.shape_from, be_liberal=False)

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
        if not value.shape == self.shape_from:
            msg = ('We were given value with shape %s instead of '
                   'expected %s.' % (value.shape, self.shape_from))
            if not self.be_liberal:
                raise ValueError(msg)
            else:
                if not self.warned:
                    self.warned = True
                    self.error(msg)

        # assert value.shape == self.shape_from
        from scipy.misc import imresize
        y = imresize(value, self.shape_to, mode='F')
        y = np.array(y, dtype='float64')      
        warnings.warn('Hardcoded float64')
        assert y.min() >= self.vmin
        assert y.max() <= self.vmax
        return y

    def __repr__(self):
        return 'Resample({})'.format(self.shape_to)


# def scipy_image_resample_01(image, shape, order=3):
#     """ clips in 0 1 """
#     y = scipy_image_resample(image, shape, order)
#     np.clip(y, 0, 1, y)
#     return y

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

