from contracts import contract

from bootstrapping_olympics import (RepresentationNuisance, NuisanceNotInvertible,
    check_streamels_1D)


__all__ = ['Subsample']


class Subsample(RepresentationNuisance):
    ''' 
        Selects a fraction of the sensels.
    '''

    @contract(every='int,>1')
    def __init__(self, every):
        """
            :param every: interval; 2 selects every other sensel
        """
        self.every = every

    def inverse(self):
        raise NuisanceNotInvertible()

    def transform_streamels(self, streamels):
        check_streamels_1D(streamels)
        
        n = streamels.size
        self.select = range(0, n, self.every)
        streamels2 = streamels[self.select]
        return streamels2

    def transform_value(self, value):
        return value[self.select]

    def __repr__(self):
        return 'Subsample(%s)' % self.every


