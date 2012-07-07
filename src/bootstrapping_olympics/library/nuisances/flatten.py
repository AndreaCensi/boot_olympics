from . import check_streamels_1D, contract
from bootstrapping_olympics import UnsupportedSpec, RepresentationNuisance

__all__ = ['Flatten', 'Reshape', 'To2D']


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
        return Reshape((self.cur_size,), self.cur_shape)

    @contract(value='array', returns='array[N]')
    def transform_value(self, value):
        ''' Flattens the given value. '''
        return value.flatten()

    def __repr__(self):
        return 'Flatten()'


class Reshape(RepresentationNuisance):
    ''' Reshape the values to a given shape. '''

    @contract(shape_from='seq[>=1](int)', shape_to='seq[>=1](int)')
    def __init__(self, shape_from, shape_to):
        ''' 
            The two shapes should have the same number of elements. 
        
            :param shape_from: The shape the data is expected to be.
            :param shape_to: The new shape for the data.
        '''
        # TODO: check same number of elements
        self.shape_from = shape_from
        self.shape_to = shape_to

    def inverse(self):
        return Reshape(self.shape_to, self.shape_from)

    def transform_streamels(self, streamels):
        cur_shape = streamels.shape
        if cur_shape != self.shape_from:
            msg = ('Cannot apply %s to array of size %s.' % 
                   (self, cur_shape))
            raise UnsupportedSpec(msg)

        return streamels.reshape(self.shape_to)

    def transform_value(self, value):
        # TODO: check value shape
        return value.reshape(self.shape_to)

    def __repr__(self):
        return 'Reshape({}, {})'.format(self.shape_from, self.shape_to)


class To2D(RepresentationNuisance):
    ''' 
        Transforms a stream of shape ``(n)`` 
        into one of shape ``(1,n)``. 
    ''' 

    def inverse(self):
        ''' The inverse is the :py:class:`Flatten` nuisance. '''
        return Flatten()

    def transform_streamels(self, streamels):
        check_streamels_1D(streamels)
        streamels2 = streamels.reshape((1, streamels.size))
        return streamels2

    def transform_value(self, value):
        return value.reshape((1, value.size))

    def __repr__(self):
        return 'To2D()'

    
class To2Dm(RepresentationNuisance):
    ''' 
        Transforms a stream of shape ``(n)`` 
        into one of shape ``(M,n)``, with default M=1. 
        
    '''
    
    @contract(M='int,>=1')
    def __init__(self, M):
        self.M = M

    def inverse(self):
        ''' The inverse is the :py:class:`Flatten` nuisance. '''
        return Flatten()

    def transform_streamels(self, streamels):
        check_streamels_1D(streamels)
        streamels2 = streamels.reshape((1, streamels.size))
        return streamels2

    def transform_value(self, value):
        return value.reshape((1, value.size))

    def __repr__(self):
        return 'To2D()'
