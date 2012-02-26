from . import check_streamels_1D, check_streamels_continuous, contract, np
from ... import (StreamSpec, UnsupportedSpec, RepresentationNuisance,
    ValueFormats)

__all__ = ['GLNuisance']


# TODO: use generic

class GLNuisance(RepresentationNuisance):
    ''' A general linear transformation. '''

    @contract(A='seq[N](seq[N])')
    def __init__(self, A):
        self.A = np.array(A)

    def inverse(self):
        return GLNuisance(np.linalg.inv(self.A))

    @contract(stream_spec=StreamSpec)
    def transform_spec(self, stream_spec):
        streamels = stream_spec.get_streamels()
        check_streamels_1D(streamels)
        check_streamels_continuous(streamels)

        if self.A.shape[1] != streamels.shape[0]:
            msg = ('Matrix of shape %r cannot act on stream of shape %s' %
                   (self.A.shape, streamels.shape))
            raise UnsupportedSpec(msg)

        streamels = stream_spec.get_streamels()
        streamels2 = streamels.copy()
        norm = np.abs(self.A).sum() # XXX
        streamels2['kind'][:] = ValueFormats.Continuous
        streamels2['lower'] *= norm
        streamels2['upper'] *= norm
        streamels2['default'] = self.transform_value(streamels['default'])

        stream_spec2 = StreamSpec(id_stream=stream_spec.id_stream,
                                  streamels=streamels2,
                                  extra={},
                                  filtered={},
                                  desc=stream_spec.desc)

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


