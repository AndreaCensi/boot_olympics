from contracts import contract
import numpy as np
from .bits_utils import (BLOCK_SIZE, NBITS_IN_BYTE, bits2string,
               string2bits, nbytes_to_hold, PAD_CHAR)

__all__ = ['bits_decrypt', 'bits_encrypt',
           'bits_pad_to_byte', 'string_pad_to_block_size']

@contract(bits='seq[N](0|1)', returns='array[X](=0|=1),X>=N,X<=N+7')
def bits_pad_to_byte(bits):
    nmissing_bits = ((NBITS_IN_BYTE - len(bits) % NBITS_IN_BYTE)
                     % NBITS_IN_BYTE)
    # print('e:adding %d padding bits' % nmissing_bits)
    bits = list(bits) + [0] * nmissing_bits
    return np.array(bits, dtype='int')


def string_pad_to_block_size(s, block_size):
    nextra_bytes = (block_size - len(s) % block_size) % block_size
    return s + PAD_CHAR * nextra_bytes


@contract(bits='seq[N](0|1)', password='str',
          returns='array[X](=0|=1),X>=N,X<=N+63')
def bits_encrypt(bits, password):
    from Crypto.Cipher import DES
    obj = DES.new(password, DES.MODE_ECB)
    # print('e:starting with %d bits' % len(bits))
    # pad the bits to who bytes
    bits = bits_pad_to_byte(bits)
    assert len(bits) % NBITS_IN_BYTE == 0
    plain = bits2string(bits)
    # print('e:Plain message: %r' % plain)
    plain = string_pad_to_block_size(plain, BLOCK_SIZE)
    # print('e:Plain (padded): %r' % plain)
    # encrypt the string
    cypher = obj.encrypt(plain)
    # convert to bits
    bits = string2bits(cypher)
    return np.array(bits)


@contract(bits='seq[N](0|1)', password='str',
          ngoodbits='int,X,>0,<=N',
          returns='array[X](=0|=1)')
def bits_decrypt(bits, password, ngoodbits):
    from Crypto.Cipher import DES
    obj = DES.new(password, DES.MODE_ECB)
    # convert to string
    cypher = bits2string(bits)
    # decrypt
    plain = obj.decrypt(cypher)
    # print('d:Plain (padded): %r' % plain)
    # extra stuff
    ngood_bytes = nbytes_to_hold(ngoodbits)
    nextra_bytes = len(plain) - ngood_bytes
    # print('d:removing %d pad chars' % nextra_bytes)
    extra_chars = plain[-nextra_bytes:]
    assert len(extra_chars) == nextra_bytes
    if extra_chars != PAD_CHAR * nextra_bytes:
        msg = ("While decrypting, obtained plain %r of len %d. "
               "I'm looking for %d bits, "
               "which takes %d bytes, giving %d extra bytes. The extra "
               "stuff is %r, which is not all %r." % 
               (plain, len(plain), ngoodbits, ngood_bytes, nextra_bytes,
                extra_chars, PAD_CHAR))
        raise ValueError(msg)
    interesting_chars = plain[:-nextra_bytes]
    interesting_bits = string2bits(interesting_chars)
    goodbits = interesting_bits[:ngoodbits]
    return np.array(goodbits)



