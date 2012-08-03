from . import contract, UnsupportedSpec, RepresentationNuisance


__all__ = ['Reshape']


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

