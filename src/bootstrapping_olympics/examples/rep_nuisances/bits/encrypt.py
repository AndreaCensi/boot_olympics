from . import (bits_encrypt, bits_decrypt, contract, np, nbytes_to_encrypt,
    NBITS_IN_BYTE)
from .... import (StreamSpec, UnsupportedSpec, streamels_all_of_kind,
    ValueFormats, streamel_dtype, RepresentationNuisance)

__all__ = ['Encrypt', 'Decrypt']


def check_1d_bit_sequence(streamels, who):
    ''' Checks that it is a 1D sequence of bits (integers in {0,1}). '''
    if not streamels_all_of_kind(streamels, ValueFormats.Discrete):
        msg = '%s only supports discrete streams.' % who
        raise UnsupportedSpec(msg)

    if streamels.ndim != 1: # XXX
        msg = '%s only works with 2D streams.' % who
        raise UnsupportedSpec(msg)

    if (np.any(streamels['lower'] != 0) or
         np.any(streamels['upper'] != 1)):
        msg = '%s expects only bits as input.' % who
        raise UnsupportedSpec(msg)


class Encrypt(RepresentationNuisance):
    """ 
        This nuisance encrypts the bits.
    
        The output is padded to 8 bits.
    """

    def __init__(self, password):
        self.password = password

    @contract(stream_spec=StreamSpec)
    def transform_spec(self, stream_spec):
        streamels = stream_spec.get_streamels()
        check_1d_bit_sequence(streamels, 'Encrypt')

        self.nbits = streamels.size
        bytes_required = nbytes_to_encrypt(self.nbits)
        bits_required = bytes_required * NBITS_IN_BYTE

        streamels2 = np.zeros(bits_required, streamel_dtype)
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

    def inverse(self):
        return Decrypt(self.password, self.nbits)

    @contract(value='array[N](=0|=1)',
              returns='array[X](=0|=1),X>=N,X<=N+64') # XXX
    def transform_value(self, value):
        return bits_encrypt(value, self.password)


class Decrypt(RepresentationNuisance):

    def __init__(self, password, ngoodbits):
        self.password = password
        self.ngoodbits = ngoodbits
        bytes_required = nbytes_to_encrypt(ngoodbits)
        self.expect_nbits = bytes_required * NBITS_IN_BYTE

    @contract(stream_spec=StreamSpec)
    def transform_spec(self, stream_spec):
        streamels = stream_spec.get_streamels()
        check_1d_bit_sequence(streamels, 'Decrypt')

        if streamels.size != self.expect_nbits:
            msg = 'I expect %d bits to decrypt.' % self.expect_nbits
            raise UnsupportedSpec(msg)

        streamels2 = np.zeros(self.ngoodbits, streamel_dtype)
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

    def inverse(self):
        return Encrypt(self.password)

    @contract(value='array[N](=0|=1)',
              returns='array[X](=0|=1),X<=N,X>=N-64') # XXX
    def transform_value(self, value):
        return bits_decrypt(value, self.password, self.ngoodbits)


