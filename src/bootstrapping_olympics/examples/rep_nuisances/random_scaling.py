from . import contract, np
from ...interfaces import StreamSpec, UnsupportedSpec, RepresentationNuisance

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
        if len(stream_spec.shape()) != 1:
            msg = 'RandomScaling only supports 1D streams.'
            raise UnsupportedSpec(msg)

        np.random.seed(self.seed)
        n = stream_spec.size()
        self.scale = np.random.exponential(scale=1.0, size=n)

#        print('seed %s scale %s' % (self.seed, self.scale)) 
        if self.inverted:
            self.scale = 1.0 / self.scale

        self.scale = self.scale.astype('float32')

        # TODO: default values        
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

        # Doesn't change the spec
        return stream_spec2

    def transform_value(self, values):
        if self.scale is None:
            raise ValueError('Please call transform_spec() first.')
        return values * self.scale

    def __str__(self):
        if self.scale is None:
            data = ""
        else:
            data = ',%s' % self.scale
        return 'Scale(%s,%s%s)' % (self.seed, self.inverted, data)


