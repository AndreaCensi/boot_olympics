from contracts import contract
from bootstrapping_olympics import UnsupportedSpec, RepresentationNuisance

__all__ = ['Flatten']


class Flatten(RepresentationNuisance):
    ''' A nuisance that flattens the streamels. '''

    def __init__(self):
        self.cur_shape = None

    def transform_streamels(self, streamels):
        if len(streamels.shape) == 1:
            msg = 'Flatten only works with non-1D streams.'
            raise UnsupportedSpec(msg)

        self.cur_shape = streamels.shape
        self.cur_size = streamels.size

        streamels2 = streamels.flatten()

        return streamels2

    def inverse(self):
        ''' The inverse operation is a :py:class:`Reshape` operation. '''
        from . import Reshape
        return Reshape((self.cur_size,), self.cur_shape)

    @contract(value='array', returns='array[N]')
    def transform_value(self, value):
        ''' Flattens the given value. '''
        return value.flatten()

    def __repr__(self):
        return 'Flatten()'


