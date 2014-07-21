from contracts import contract

from bootstrapping_olympics import (UnsupportedSpec, RepresentationNuisance)
import numpy as np
from streamels import streamels_all_of_kind, ValueFormats, streamel_dtype, check_streamels_2D

from .bits_utils import value2bits
from .graycode import gray
import warnings


__all__ = ['Int2bits', 'Bits2int']


def format_word(word):
    return tuple(int(b) for b in word)


def format_code(code):
    return [format_word(x) for x in code]


@contract(max_value='int,>0', returns='list(seq[N](0|1))')
def base2code(max_value):
    nbits = int(np.ceil(np.log2(max_value)))
    nbits = max(1, nbits)  # max_value=1
    #    print('computing %d bits base2 code for max_value = %s' %
    #          (nbits, max_value))
    max_value = 2 ** nbits
    code = [value2bits(i, nbits) for i in range(max_value)]
    return format_code(code)


@contract(max_value='int,>0', returns='list(seq[N](0|1))')
def graycode(max_value):
    nbits = int(np.ceil(np.log2(max_value)))
    nbits = max(1, nbits)  # max_value=1
    #    print('computing %d bits gray code for max_value = %s' %
    #          (nbits, max_value))
    code = gray(nbits)
    return format_code(code)

allowed_codes = {'base2': base2code, 'gray': graycode}


class Int2bits(RepresentationNuisance):
    ''' 
        This nuisance represents integer values with
        their bit representation (0-1).   
        
        Converts a M array into an MxN array, 
        using N bits, where N is the minimum necessary to
        represent all the numbers in the array.
    '''

    def __init__(self, code):
        self.nbits = None
        if not code in allowed_codes:
            msg = ('Unknown code %r, choose in %s' % 
                   (code, allowed_codes.keys()))
            raise ValueError(msg)
        self.codefunc = allowed_codes[code]
        self.code = code

    def transform_streamels(self, streamels):
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

        # print('upper: %s' % self.upper)
        self.max_value = int(np.max(self.upper))
        self.codewords = self.codefunc(self.max_value)
        assert len(self.codewords) >= self.max_value
        self.nbits = len(self.codewords[0])

        self.old_shape = streamels.shape
        self.new_shape = (streamels.size, self.nbits)
        streamels2 = np.zeros(self.new_shape, streamel_dtype)
        streamels2['kind'] = ValueFormats.Discrete
        streamels2['lower'] = 0
        streamels2['upper'] = 1
        streamels2['default'] = self.transform_value(streamels['default'])
        return streamels2

    @contract(value='array[N]',  # TODO: add generic int
              returns='array[NxM](=0|=1)')
    def transform_value(self, value):
        assert value.shape == self.old_shape
        value2 = np.zeros(self.new_shape, 'int')
        for i, v in enumerate(value):
            # TODO: check value
            value2[i, :] = self.codewords[int(v)]
        return value2

    def inverse(self):
        warnings.warn(' we cannot really recover lower and upper') # XXX:
        # if lower !=0 or upper is not a power of 2 (and all the same)
        return Bits2int(self.code)

    def left_inverse(self):
        return self.inverse()



    def __repr__(self):
        return 'Int2bits(%r)' % self.code


class Bits2int(RepresentationNuisance):
    ''' 
        This nuisance represents transforms back 
        a bit representation into integer values.
        
        Converts an MxN array (N: number of bits), to
        an M array.
    '''

    def __init__(self, code):
        if not code in allowed_codes:
            msg = ('Unknown code %r, choose in %s' % 
                   (code, allowed_codes.keys()))
            raise ValueError(msg)
        self.code = code
        self.codefunc = allowed_codes[code]

    def transform_streamels(self, streamels):
        check_streamels_2D(streamels)
        shape = streamels.shape
        # TODO: use normal function
        if not streamels_all_of_kind(streamels, ValueFormats.Discrete):
            msg = 'Bits2int only supports discrete streams.'
            raise UnsupportedSpec(msg)

        if (np.any(streamels['lower'] != 0) or
             np.any(streamels['upper'] != 1)):
            msg = 'I expect only bits as input.'
            raise UnsupportedSpec(msg)

        nvalues, nbits = shape

        max_value = int(2 ** nbits - 1)
        self.codewords = self.codefunc(max_value)
        self.codewords_inv = {}
        for i, x in enumerate(self.codewords):
            self.codewords_inv[x] = i
        # print('codewords: %s' % self.codewords)

        streamels2 = np.zeros(nvalues, streamel_dtype)
        streamels2['kind'] = ValueFormats.Discrete
        streamels2['lower'] = 0
        streamels2['upper'] = 2 ** nbits - 1
        streamels2['default'] = self.transform_value(streamels['default'])
        return streamels2

    @contract(value='array[NxM](=0|=1)', returns='array[N]')  # XXX
    def transform_value(self, value):
        value2 = np.zeros(value.shape[0], 'int')
        for i, bits in enumerate(value):
            word = format_word(bits)
            value2[i] = self.codewords_inv[word]
        return value2

    def inverse(self):
        return Int2bits(self.code)

    def left_inverse(self):
        return self.inverse()

    def __repr__(self):
        return 'Bits2int(%r)' % self.code
