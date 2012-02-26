from . import string2bits, bits2string


def test_conversions():
    s = 'hello world'
    bits = string2bits(s)
    s2 = bits2string(bits)
    assert s == s2

