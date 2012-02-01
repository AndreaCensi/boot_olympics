from . import contract, np
from ... import (StreamSpec, UnsupportedSpec, RepresentationNuisance,
    streamels_all_of_kind)

__all__ = ['RandomScaling']


class RandomScaling(RepresentationNuisance):
    ''' A fixed random scaling of the signal. '''

    @contract(seed='int', inverted='bool')
    def __init__(self, seed, inverted=False):
        self.seed = seed
        self.inverted = inverted
        self.scale = None

    def inverse(self):
        return RandomScaling(self.seed, not self.inverted)

    @contract(stream_spec=StreamSpec)
    def transform_spec(self, stream_spec):
        # FIXME: check this only for float values

        if not streamels_all_of_kind(streamels=stream_spec.get_streamels(),
                                     kind='C'): #XXX
            msg = 'RandomScaling only supports continuous streams.'

        if len(stream_spec.shape()) != 1:
            msg = 'RandomScaling only supports 1D streams.'
            raise UnsupportedSpec(msg)

        np.random.seed(self.seed)
        n = stream_spec.size()
        self.scale = np.random.exponential(scale=1.0, size=n)

        if self.inverted:
            self.scale = 1.0 / self.scale

        streamels = stream_spec.get_streamels()
        streamels2 = streamels.copy()
        streamels2['lower'] *= self.scale
        streamels2['upper'] *= self.scale
        streamels2['default'] *= self.scale

        filtered = {
            'original': [stream_spec.to_yaml()],
            'filter': ['bootstrapping_olympics.examples.RandomScaling',
                       {'seed':self.seed, 'inverted': self.inverted}]
        }
        stream_spec2 = StreamSpec(id_stream=stream_spec.id_stream,
                                  streamels=streamels2,
                                  extra={},
                                  filtered=filtered,
                                  desc="%s (scaled)" % stream_spec.desc)

        # Save this so we can enforce it later
        self.lower = streamels2['lower'].copy()
        self.upper = streamels2['upper'].copy()

        return stream_spec2

    def transform_value(self, values):
        if self.scale is None:
            raise ValueError('Please call transform_spec() first.')

        value2 = values * self.scale
        # enforce upper/lower bounds, to account for numerical errors
        value2 = value2.astype('float32')
        value2 = np.minimum(self.upper, value2)
        value2 = np.maximum(self.lower, value2)
        return value2

    def __str__(self):
        if self.scale is None:
            data = ""
        else:
            data = ',%s' % self.scale
        return 'Scale(%s,%s%s)' % (self.seed, self.inverted, data)


