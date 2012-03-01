from . import (check_streamels_1D, check_streamels_continuous, np,
    check_streamels_range)
from bootstrapping_olympics import (NuisanceNotInvertible, streamel_dtype,
     RepresentationNuisance, ValueFormats)


class NormalizeMin(RepresentationNuisance):
    ''' 
        Normalizes the streamels into a ``[0,1]`` range by removing the 
        minimum and linearly scaling.
        
        Transforms: ::
        
            z[:n] = (y - min(y)) /  max(y) - min(y) 
            z[n]  = min(y) 
            z[n+1]= max(y) 
            
        To make this invertible, the streamels must have range in [0,1].
    '''

    def inverse(self):
        return NormalizeMinInverse()

    def transform_streamels(self, streamels):
        check_streamels_1D(streamels)
        check_streamels_continuous(streamels)
        check_streamels_range(streamels, 0, 1)

        n = streamels.size
        streamels2 = np.zeros(n + 2, streamel_dtype)
        streamels2['kind'][:] = ValueFormats.Continuous
        streamels2['lower'][:n] = 0
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
        vmin = np.min(value)
        vmax = np.max(value)
        spread = vmax - vmin

        n = value.size
        value2 = np.zeros(n + 2)
        if spread > 0:
            normalized = (value - vmin) / spread
        else:
            normalized = (value - vmax)
        value2[:n] = normalized
        value2[n] = vmin
        value2[n + 1] = vmax

        # TODO: check we are not clipping too much
        # enforce upper/lower bounds, to account for numerical errors
        return np.clip(value2.astype('float32'), self.lower, self.upper)

    def __repr__(self):
        return 'NormalizeMin()'


class NormalizeMinInverse(RepresentationNuisance):
    ''' 
        This is the inverse of NormalizeMin.
    '''

    def __init__(self):
        # TODO: memorize old bounds
        pass

    def inverse(self):
        raise NuisanceNotInvertible('Not implemented')

    def transform_streamels(self, streamels):
        check_streamels_1D(streamels)
        check_streamels_continuous(streamels)
        check_streamels_range(streamels, 0.0, 1.0)

        M = streamels.size
        n = M - 2
        streamels2 = np.zeros(n, streamel_dtype)
        streamels2['kind'][:] = ValueFormats.Continuous
        streamels2['lower'][:] = 0
        streamels2['upper'][:] = +1

        streamels2['default'] = self.transform_value(streamels['default'])

        return streamels2

    def transform_value(self, value):
        z = value[:-2]
        vmin = value[-2]
        vmax = value[-1]
        spread = vmax - vmin

        y = z * spread + vmin

        # TODO: check we are not clipping too much
        # enforce upper/lower bounds, to account for numerical errors
        return np.clip(y.astype('float32'), 0, 1)

    def __repr__(self):
        return 'NormalizeMinInverse()'
