from bootstrapping_olympics import  RepresentationNuisance
from streamels import check_streamels_1D

from .flatten import Flatten


__all__ = ['To2D']


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


