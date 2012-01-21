from . import np
from ...interfaces import (RepresentationNuisance, StreamSpec, streamel_dtype,
    ValueFormats, UnsupportedSpec)
from .popcode_functions import popcode, popcode_inv


class PopCode(RepresentationNuisance):
    ''' 
        This nuisance transforms a 1D representation of the signal
        into a 2D representation. 
    '''

    def __init__(self, resolution=128):
        self.resolution = resolution

    def inverse(self):
        return PopCodeInv()

    def transform_spec(self, stream_spec):
        shape = stream_spec.shape()
        streamels = stream_spec.get_streamels()

        if len(shape) != 1:
            raise UnsupportedSpec("I only accept 1D signals.")

        if not np.all(streamels['kind'] == ValueFormats.Continuous):
            raise UnsupportedSpec("I only accept continuous signals.")

        if not np.all(streamels['lower'] == 0):
            raise UnsupportedSpec("I only accept signals with lower=0.")

        if not np.all(streamels['upper'] == 1):
            raise UnsupportedSpec("I only accept signals with upper=1.")

        # TODO: check all floats and in 0--1

        shape2 = shape[0], self.resolution
        streamels2 = np.empty(shape=shape2, dtype=streamel_dtype)
        streamels2['kind'].flat[:] = ValueFormats.Continuous
        streamels2['lower'].flat[:] = 0
        streamels2['upper'].flat[:] = 1
        streamels2['default'] = popcode(stream_spec.get_default_value(),
                                       self.resolution)

#        print 'old default', streamels['default']
#        print 'new default', streamels2['default']

        return StreamSpec(id_stream=None, # XXX
                           streamels=streamels2,
                           extra=None # XXX
                           )

    def transform_value(self, values):
        return popcode(values, self.resolution)

    def __repr__(self):
        return 'PopCode(resolution=%d)' % self.resolution


class PopCodeInv(RepresentationNuisance):
    """ Inverse of PopCode """

    def inverse(self):
        return PopCode() # XXX: cannot pass resolution

    def transform_spec(self, stream_spec):
        shape = stream_spec.shape()
        streamels = stream_spec.get_streamels()
        if len(shape) != 2:
            raise UnsupportedSpec("I only accept 2D signals.")

        if not np.all(streamels['kind'] == ValueFormats.Continuous):
            raise UnsupportedSpec("I only accept continuous signals.")

        if not np.all(streamels['lower'] == 0):
            raise UnsupportedSpec("I only accept signals with lower=0.")

        if not np.all(streamels['upper'] == 1):
            raise UnsupportedSpec("I only accept signals with upper=1.")

        N, _ = shape
        shape2 = N,
        streamels2 = np.empty(shape=shape2, dtype=streamel_dtype)
        streamels2['kind'].flat[:] = ValueFormats.Continuous
        streamels2['lower'].flat[:] = 0
        streamels2['upper'].flat[:] = 1
        default = popcode_inv(stream_spec.get_default_value())
        streamels2['default'] = default.astype('float32')

#        print 'old default', streamels['default']
#
#        print 'new default', streamels2['default']

        return StreamSpec(id_stream=None, # XXX
                           streamels=streamels2,
                           extra=None # XXX
                           )

    def transform_value(self, values):
#        print("inverting %s" % values)
        return popcode_inv(values.astype('float32'))

    def __repr__(self):
        return 'PopCodeInv()'
