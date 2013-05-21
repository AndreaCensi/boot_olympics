from . import bits_encrypt, bits_decrypt
from bootstrapping_olympics.utils import assert_allclose
import numpy as np


def test_encryption():
    password = 'ciaociao'
    for nbits in [8, 16, 20]:
        bits = np.round(np.random.rand(nbits))
        msg = bits_encrypt(bits, password)
        bits2 = bits_decrypt(msg, password, len(bits))
        assert_allclose(bits, bits2)
