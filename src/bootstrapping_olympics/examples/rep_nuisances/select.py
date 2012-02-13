from . import contract, np
from ... import (StreamSpec, UnsupportedSpec, RepresentationNuisance,
 NuisanceNotInvertible)

__all__ = ['Select']


class Select(RepresentationNuisance):
    ''' 
        A non-invertible nuisance that selects some of the sensels. 
        Works only for 1D streams.
    '''

#    @contract(which='(int,>0)|seq[>0](int,>0)')
    def __init__(self, which):
        """
            Initializes this sequence.
            
            :param which: either an integer or a sequence of integers
        """
        self.which = which

        if isinstance(which, int):
            self.select = np.array(range(which))
        else:
            self.select = np.array(which)

    def inverse(self):
        raise NuisanceNotInvertible()

    @contract(stream_spec=StreamSpec)
    def transform_spec(self, stream_spec):
        shape = stream_spec.shape()
        if len(shape) != 1:
            msg = 'Select() only supports 1D streams.'
            raise UnsupportedSpec(msg)

        size = stream_spec.size()
        min_size = max(self.select) + 1
        if min_size > size:
            msg = ('This nuisance requires a stream with at least %d '
                   'streamels; found %s.' % (min_size, size))
            raise UnsupportedSpec(msg)

        streamels = stream_spec.get_streamels()
        streamels2 = streamels[self.select]

        return StreamSpec(id_stream=stream_spec.id_stream,
                                  streamels=streamels2,
                                  extra={},
                                  filtered={},
                                  desc="%s (scaled)" % stream_spec.desc)

    def transform_value(self, value):
        return value[self.select]

    def __repr__(self):
        return 'Select(%s)' % self.which


