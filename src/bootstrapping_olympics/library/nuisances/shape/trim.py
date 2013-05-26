from bootstrapping_olympics import (NuisanceNotInvertible, UnsupportedSpec,
    RepresentationNuisance)
from contracts import contract

__all__ = ['Trim']


class Trim(RepresentationNuisance):
    ''' Crops the stream along the first two dimensions. '''

    @contract(trim='seq[2](seq[2](int,>=0))')
    def __init__(self, trim):
        self.trim = trim

    def transform_streamels(self, streamels):
        if streamels.ndim < 2:
            msg = 'Flatten needs at least 2 dimensions.'
            raise UnsupportedSpec(msg)

        return self._crop(streamels)
        
    def inverse(self):
        raise NuisanceNotInvertible()

    def transform_value(self, value):
        return self._crop(value)

    def _crop(self, value):
        shape = value.shape
        xtrim = self.trim[0]
        ytrim = self.trim[1]
        xmin = xtrim[0]
        xmax = shape[0] - xtrim[1]
        ymin = ytrim[0]
        ymax = shape[1] - ytrim[1]
        
        v = value[xmin:xmax, ymin:ymax, ...]
        return v
        
    def __repr__(self):
        return 'Flatten()'


