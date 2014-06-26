from contracts import contract

from bootstrapping_olympics import (UnsupportedSpec,
    RepresentationNuisance)
import numpy as np
from streamels import  check_1d_bit_sequence, ValueFormats, streamel_dtype

from . import bits_encrypt, bits_decrypt, nbytes_to_encrypt
from .bits_utils import NBITS_IN_BYTE


__all__ = ['Encrypt', 'Decrypt']


class Encrypt(RepresentationNuisance):
    """ 
        This nuisance encrypts the bits.
    
        The output is padded to 8 bits.
    """

    def __init__(self, password):
        self.password = password

    def transform_streamels(self, streamels):
        check_1d_bit_sequence(streamels, 'Encrypt')

        self.nbits = streamels.size
        bytes_required = nbytes_to_encrypt(self.nbits)
        bits_required = bytes_required * NBITS_IN_BYTE

        streamels2 = np.zeros(bits_required, streamel_dtype)
        streamels2['kind'] = ValueFormats.Discrete
        streamels2['lower'] = 0
        streamels2['upper'] = 1
        streamels2['default'] = self.transform_value(streamels['default'])
        return streamels2

    def inverse(self):
        return Decrypt(self.password, self.nbits)

    @contract(value='array[N](=0|=1)',
              returns='array[X](=0|=1),X>=N,X<=N+64')  # XXX
    def transform_value(self, value):
        return bits_encrypt(value, self.password)


class Decrypt(RepresentationNuisance):

    def __init__(self, password, ngoodbits):
        self.password = password
        self.ngoodbits = ngoodbits
        bytes_required = nbytes_to_encrypt(ngoodbits)
        self.expect_nbits = bytes_required * NBITS_IN_BYTE

    def transform_streamels(self, streamels):
        check_1d_bit_sequence(streamels, 'Decrypt')

        if streamels.size != self.expect_nbits:
            msg = 'I expect %d bits to decrypt.' % self.expect_nbits
            raise UnsupportedSpec(msg)

        streamels2 = np.zeros(self.ngoodbits, streamel_dtype)
        streamels2['kind'] = ValueFormats.Discrete
        streamels2['lower'] = 0
        streamels2['upper'] = 1
        streamels2['default'] = self.transform_value(streamels['default'])

        return streamels2

    def inverse(self):
        return Encrypt(self.password)

    @contract(value='array[N](=0|=1)',
              returns='array[X](=0|=1),X<=N,X>=N-64')  # XXX
    def transform_value(self, value):
        return bits_decrypt(value, self.password, self.ngoodbits)


