from StringIO import StringIO
import pickle

from contracts import ContractNotRespected
from numpy.testing.utils import assert_raises


from .xstream_specs import valid_xstream_spec, invalid_xstream_spec
from streamels.stream_spec import XStreamSpec


def check_parsing(x):
    XStreamSpec.from_yaml(x)


def check_parsing_invalid(x):
    print('Checking: %s' % x)
    assert_raises((ValueError, ContractNotRespected), 
                  XStreamSpec.from_yaml, x)


def check_conversions(x):
    spec = XStreamSpec.from_yaml(x)
    assert isinstance(spec, XStreamSpec)
    y = spec.to_yaml()
    assert isinstance(y, dict)
    spec2 = spec.from_yaml(y)
    print('x: %r' % x)
    print('spec: %r' % spec)
    print('y: %r' % y)
    print('spec2: %r' % spec2)
    assert spec == spec2

def check_pickling(x):
    spec = XStreamSpec.from_yaml(x)
    sio = StringIO()
    pickle.dump(spec, sio)
    s = sio.getvalue()
    sio = StringIO(s)
    spec2 = pickle.load(sio)

    print('x: %r' % x)
    print('spec: %r' % spec)
    print('spec2: %r' % spec2)
    assert spec == spec2

def test_valid():
    for x in valid_xstream_spec:
        assert isinstance(x, dict)
        yield check_parsing, x
        yield check_conversions, x
        yield check_pickling, x


def test_invalidi():
    for x in invalid_xstream_spec:
        assert isinstance(x, dict)
        yield check_parsing_invalid, x





