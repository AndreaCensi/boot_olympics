from . import contract, np
from ... import (StreamSpec, UnsupportedSpec, RepresentationNuisance,
    streamels_all_of_kind, ValueFormats, streamel_dtype)

__all__ = ['Int2bits', 'Bits2int']


class Int2bits(RepresentationNuisance):
    ''' 
        This nuisance represents integer values with
        their bit representation (0-1).   
        
        Converts a M array into an MxN array, 
        using N bits, where N is the mininum used to
        represent all the numbers in the array.
    '''

    def __init__(self):
        self.nbits = None

    @contract(stream_spec=StreamSpec)
    def transform_spec(self, stream_spec):
        streamels = stream_spec.get_streamels()

        if not streamels_all_of_kind(streamels, ValueFormats.Discrete):
            msg = 'Int2bits only supports discrete streams.'
            raise UnsupportedSpec(msg)

        if streamels.ndim != 1:
            msg = 'Int2bits only works with 1D streams.'
            raise UnsupportedSpec(msg)

        # Save these so we can use them later
        self.lower = streamels['lower'].copy()
        self.upper = streamels['upper'].copy()

        if not np.all(self.lower >= 0):
            msg = 'I assume that numbers are positive.'
            raise UnsupportedSpec(msg)

        max_value = np.max(self.upper)
        nbits = int(np.ceil(np.log2(max_value)))
        print('Need %d bits to represent %s' % (nbits, max_value))
        self.nbits = nbits

        self.old_shape = streamels.shape
        self.new_shape = (streamels.size, nbits)
        streamels2 = np.zeros(self.new_shape, streamel_dtype)
        streamels2['kind'] = ValueFormats.Discrete
        streamels2['lower'] = 0
        streamels2['upper'] = 1
        streamels2['default'] = self.transform_value(streamels['default'])

        stream_spec2 = StreamSpec(id_stream=stream_spec.id_stream,
                                  streamels=streamels2,
                                  extra={},
                                  filtered={},
                                  desc=None)

        return stream_spec2

    @contract(value='array[N]', # TODO: add generic int
              returns='array[NxM](=0|=1)')
    def transform_value(self, value):
        assert value.shape == self.old_shape
        value2 = np.zeros(self.new_shape, 'int')
        for i, v in enumerate(value):
            value2[i, :] = value2bits(v.item(), self.nbits)
        return value2

    def inverse(self):
        # XXX: we cannot really recover lower and upper
        # if lower !=0 or upper is not a power of 2 (and all the same)
        raise Bits2int()

    def __repr__(self):
        return 'Int2bits()'


@contract(x='int,>=0', nbits='N,int,>=1', returns='array[N](=0|=1)')
def value2bits(x, nbits):
    ''' Converts an integer to its bit representation.
        LSB first. '''
    b = np.zeros(nbits, 'int')
    for i in range(nbits):
        b[i] = (int(2 ** i) & int(x)) > 0
    return b


@contract(b='array[N](=0|=1)', returns='int')
def bits2value(b):
    ''' Converts the bits back into integers. LSB first. '''
    bit_values = 2 ** np.array(range(b.size))
    return np.sum(b * bit_values)


class Bits2int(RepresentationNuisance):
    ''' 
        This nuisance represents transforms back 
        a bit representation into integer values.
        
        Converts an MxN array (N: number of bits), to
        an M array.
    '''

    def __init__(self):
        pass

    @contract(stream_spec=StreamSpec)
    def transform_spec(self, stream_spec):
        streamels = stream_spec.get_streamels()
        shape = streamels.shape
        if not streamels_all_of_kind(streamels, ValueFormats.Discrete):
            msg = 'Bits2int only supports discrete streams.'
            raise UnsupportedSpec(msg)

        if streamels.ndim != 1:
            msg = 'Bits2int only works with 2D streams.'
            raise UnsupportedSpec(msg)

        if (np.any(streamels['lower'] != 0) or
             np.any(streamels['upper'] != 1)):
            msg = 'I expect only bits as input.'
            raise UnsupportedSpec(msg)

        nvalues, nbits = shape
        streamels2 = np.zeros(nvalues, streamel_dtype)
        streamels2['kind'] = ValueFormats.Discrete
        streamels2['lower'] = 0
        streamels2['upper'] = 2 ** nbits - 1
        streamels2['default'] = self.transform_value(streamels['default'])

        stream_spec2 = StreamSpec(id_stream=stream_spec.id_stream,
                                  streamels=streamels2,
                                  extra={},
                                  filtered={},
                                  desc=None)

        return stream_spec2

    @contract(value='array[NxM](=0|=1)', returns='array[N]') # XXX
    def transform_value(self, value):
        value2 = np.zeros(value.shape[0], 'int')
        for i, B in enumerate(value):
            value2[i, :] = bits2value(B)
        return value2

    def inverse(self):
        raise Int2bits()

    def __repr__(self):
        return 'Int2bits()'



