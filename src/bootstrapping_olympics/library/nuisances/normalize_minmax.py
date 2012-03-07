from . import check_streamels_1D, check_streamels_continuous, np
from bootstrapping_olympics import (NuisanceNotInvertible, streamel_dtype,
     RepresentationNuisance, ValueFormats)

__all__ = ['NormalizeMinMax']


class NormalizeMinMax(RepresentationNuisance):
    ''' 
        Transforms: ::
        
            z[:n] = (y-mean(y)) /  max(y) - min(y)  \in -1,1
            z[n]  = mean(y)
            z[n+1]= max(y) - min(y)
    '''

    def inverse(self):
        raise NuisanceNotInvertible('Not implemented')

    def transform_streamels(self, streamels):
        check_streamels_1D(streamels)
        check_streamels_continuous(streamels)

        n = streamels.size
        streamels2 = np.zeros(n + 2, streamel_dtype)
        streamels2['kind'][:] = ValueFormats.Continuous
        streamels2['lower'][:n] = -1
        streamels2['upper'][:n] = +1

        # this is the mean
        streamels2['lower'][n] = np.mean(streamels['lower'])
        streamels2['upper'][n] = np.mean(streamels['upper'])

        # this is the spread of the data        
        streamels2['lower'][n + 1] = 0
        # XXX: we should find a different bounded  
        streamels2['upper'][n + 1] = (np.max(streamels['upper']) -
                                      np.min(streamels['lower']))

        # Save this so we can enforce it later
        self.lower = streamels2['lower'].copy()
        self.upper = streamels2['upper'].copy()

        streamels2['default'] = self.transform_value(streamels['default'])

        return streamels2

    def transform_value(self, value):
        mean = np.mean(value)
        spread = np.max(value) - np.min(value)

        n = value.size
        value2 = np.zeros(n + 2)
        if spread > 0:
            normalized = (value - mean) / spread
        else:
            normalized = (value - mean)
        value2[:n] = normalized
        value2[n] = mean
        value2[n + 1] = spread

        # enforce upper/lower bounds, to account for numerical errors
        return np.clip(value2.astype('float32'), self.lower, self.upper)

    def __repr__(self):
        return 'NormalizeMinMax()'

