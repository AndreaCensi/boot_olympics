from . import contract
from ... import (StreamSpec, UnsupportedSpec, RepresentationNuisance)

__all__ = ['Flatten', 'Reshape']


class Flatten(RepresentationNuisance):
    ''' A nuisance that flattens the streamels. '''

    def __init__(self):
        self.cur_shape = None
        pass

    @contract(stream_spec=StreamSpec)
    def transform_spec(self, stream_spec):
        if len(stream_spec.shape()) == 1:
            msg = 'Flatten only works with non-1D streams.'
            raise UnsupportedSpec(msg)

        streamels = stream_spec.get_streamels()
        streamels2 = streamels.flatten()

        stream_spec2 = StreamSpec(id_stream=stream_spec.id_stream,
                                  streamels=streamels2,
                                  extra={},
                                  filtered={},
                                  desc="%s (flatened)" % stream_spec.desc)

        self.cur_shape = stream_spec.shape()
        self.cur_size = stream_spec.size()

        return stream_spec2

    def inverse(self):
        return Reshape((self.cur_size,), self.cur_shape)

    def transform_value(self, value):
        return value.flatten()

    def __repr__(self):
        return 'Flatten()'


class Reshape(RepresentationNuisance):

    def __init__(self, shape_from, shape_to):
        self.shape_from = shape_from
        self.shape_to = shape_to

    def inverse(self):
        return Reshape(self.shape_to, self.shape_from)

    @contract(stream_spec=StreamSpec)
    def transform_spec(self, stream_spec):
        cur_shape = stream_spec.shape()
        if cur_shape != self.shape_from:
            msg = ('Cannot apply %s to array of size %s.' %
                   (self, cur_shape))
            raise UnsupportedSpec(msg)

        streamels = stream_spec.get_streamels()
        streamels2 = streamels.reshape(self.shape_to)

        stream_spec2 = StreamSpec(id_stream=stream_spec.id_stream,
                                  streamels=streamels2,
                                  extra={},
                                  filtered={},
                                  desc=stream_spec.desc # XXX
                                  )
        return stream_spec2

    def transform_value(self, value):
        return value.reshape(self.shape_to)

    def __repr__(self):
        return 'Reshape({}, {})'.format(self.shape_from, self.shape_to)
