from . import contract, np

BLOCK_SIZE = 8
PAD_CHAR = '?'
NBITS_IN_BYTE = 8


def nbytes_to_hold(nbits):
    ''' 
        Returns the number of bytes necessary to hold the given number
        of bits. 
    '''
    return int(np.ceil(float(nbits) / NBITS_IN_BYTE))


def nbytes_to_encrypt(nbits):
    ''' 
        Returns the number of bytes necessary to hold the given number
        of bits. Takes into account padding of the message to 8 bits. 
    '''
    nbytes = nbytes_to_hold(nbits)
    nblocks = int(np.ceil(float(nbytes) / BLOCK_SIZE))
    return nblocks * BLOCK_SIZE


@contract(x='int,>=0', nbits='N,int,>=1', returns='array[N](=0|=1)')
def value2bits(x, nbits):
    ''' Converts an integer to its bit representation.
        LSB first. '''
    b = np.zeros(nbits, 'int')
    for i in range(nbits):
        b[i] = (int(2 ** i) & int(x)) > 0
    return b


@contract(b='seq[N](=0|=1)', returns='int')
def bits2value(b):
    ''' Converts the bits back into integers. LSB first. '''
    bit_values = 2 ** np.array(range(len(b)))
    return int(np.sum(b * bit_values))


@contract(s='seq[N](int,>=0,<256)', returns='seq[8*N](0|1)')
def bytes2bits(s):
    ''' Returns the bits in a string '''
    bits = []
    for b in s:
        bits.extend(value2bits(b, 8))
    return np.array(bits)


@contract(bits='seq[>0](0|1)')
def bits2bytes(bits):
    ''' Packs the bits in a string, padding at 0. '''
    L = len(bits)
    if L % 8 != 0:
        msg = 'I expect a sequence of length divisible by 8, got %s.' % L
        msg += ' %s' % bits
        raise ValueError(msg)
    nbytes = len(bits) / 8
    res = []
    for i in xrange(nbytes):
        # bits for the i-th byte
        bi = bits[i * 8:(i + 1) * 8]
        res.append(bits2value(bi))
    return np.array(res)


@contract(s='str', returns='seq(0|1)')
def string2bits(s):
    values = [ord(x) for x in s]
    return bytes2bits(values)


@contract(returns='str', bits='seq(0|1)')
def bits2string(bits):
    values = bits2bytes(bits)
    return ''.join(chr(x) for x in values)




