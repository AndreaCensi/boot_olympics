from . import contract, np
from ... import (StreamSpec, UnsupportedSpec, RepresentationNuisance,
    streamels_all_of_kind, ValueFormats)

__all__ = ['GLNuisance']


class GLNuisance(RepresentationNuisance):
    ''' A general linear transformation. '''

    @contract(A='seq[N](seq[N])')
    def __init__(self, A):
        self.A = np.array(A)

    def inverse(self):
        return GLNuisance(np.linalg.inv(self.A))

    @contract(stream_spec=StreamSpec)
    def transform_spec(self, stream_spec):

        if not streamels_all_of_kind(streamels=stream_spec.get_streamels(),
                                     kind=ValueFormats.Continuous):
            msg = 'GLNuisance only supports continuous streams.'
            raise UnsupportedSpec(msg)

        shape = stream_spec.shape()
        if len(shape) != 1:
            msg = 'GLNuisance only supports 1D streams.'
            raise UnsupportedSpec(msg)

        if self.A.shape[1] != shape[0]:
            msg = ('Matrix of shape %r cannot act on stream of shape %s' %
                   (self.A.shape, shape))
            raise UnsupportedSpec(msg)

        streamels = stream_spec.get_streamels()
        streamels2 = streamels.copy()
        norm = np.abs(self.A).sum() # XXX
        streamels2['lower'] *= norm
        streamels2['upper'] *= norm
        streamels2['default'] = np.dot(self.A, streamels['default'])

        filtered = {
            'original': [stream_spec.to_yaml()],
            'filter': ['bootstrapping_olympics.examples.GLNuisance',
                       {'A': self.A.tolist()}]
        }
        stream_spec2 = StreamSpec(id_stream=stream_spec.id_stream,
                                  streamels=streamels2,
                                  extra={},
                                  filtered=filtered,
                                  desc="%s (scaled)" % stream_spec.desc)

        # Save this so we can enforce it later
        self.lower_old = streamels['lower'].copy()
        self.upper_old = streamels['upper'].copy()
        self.lower = streamels2['lower'].copy()
        self.upper = streamels2['upper'].copy()

        return stream_spec2

    def transform_value(self, value):
        value2 = np.dot(self.A, value)
        # enforce upper/lower bounds, to account for numerical errors
        value2 = value2.astype('float32')
        value2 = np.clip(value2, self.lower, self.upper)
        return value2

    def __repr__(self):
        return 'GLNuisance(%s)' % self.A.tolist()


