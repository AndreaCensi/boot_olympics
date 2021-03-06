from bootstrapping_olympics import (NuisanceNotInvertible, RepresentationNuisance,
    new_streamels, check_streamels_1D)
from contracts import contract
import numpy as np


__all__ = ['ReplicateRows', 'ExtractRow']


class ReplicateRows(RepresentationNuisance):
    ''' 
        Transforms a stream of shape ``(n)`` into one of shape ``(rows,n)``
        by replicating the rows. 
    '''
    
    @contract(rows='int,>=1')
    def __init__(self, rows):
        self.rows = rows

    def inverse(self):
        ''' The inverse is the :py:class:`Flatten` nuisance. '''
        return ExtractRow()

    def transform_streamels(self, streamels):
        check_streamels_1D(streamels)
        self.shape2 = (self.rows, streamels.size)
        streamels2 = new_streamels(self.shape2)
        for row in range(self.rows):
            streamels2[row, :] = streamels
        return streamels2

    def transform_value(self, value):
        # FIXME: will not work if not all floats 
        value2 = np.zeros(self.shape2)
        for row in range(self.rows):
            value2[row, :] = value
        return value2

    def __repr__(self):
        return 'ReplicateRows(%s)' % self.rows


class ExtractRow(RepresentationNuisance):
    """ Only useful as the inverse of To2Dm. """
    
    def inverse(self):
        raise NuisanceNotInvertible()

    def transform_streamels(self, streamels):
        # TODO: check all the same 
        return streamels[0, :]
    
    def transform_value(self, value):
        return value[0, :]
    
    def __repr__(self):
        return "ExtractRow()"
    
    
    
    
