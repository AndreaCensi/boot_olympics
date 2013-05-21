# Taken from:
# http://stackoverflow.com/questions/4119617/code-golf-gray-code
import numpy as np
from contracts import contract

__all__ = ['gray']

@contract(n='int,>0')
def _gray(n):
    if n == 1:
        return [0, 1]
    else:
        p = _gray(n - 1)
        pr = [x + (1 << (n - 1)) for x in p[::-1]]
        return p + pr


@contract(n='int,>0')
def gray(n):
    strings = [("0" * n + bin(a)[2:])[-n:] for a in _gray(n)]

    def string2bits(s):
        return np.array([int(c) for c in s])

    return [string2bits(s) for s in strings]

