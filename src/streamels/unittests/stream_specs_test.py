from StringIO import StringIO
import pickle

from contracts import ContractNotRespected
from numpy.testing.utils import assert_raises, assert_allclose

from streamels import StreamSpec

from .stream_specs import valid_stream_spec, invalid_stream_spec


def check_parsing(x):
    StreamSpec.from_yaml(x)


def check_parsing_invalid(x):
    print('Checking: %s' % x)
    assert_raises((ValueError, ContractNotRespected), StreamSpec.from_yaml, x)


def check_conversions(x):
    spec = StreamSpec.from_yaml(x)
    assert isinstance(spec, StreamSpec)
    y = spec.to_yaml()
    assert isinstance(y, dict)
    spec2 = spec.from_yaml(y)
    assert spec == spec2


def check_pickling(x):
    spec = StreamSpec.from_yaml(x)
    sio = StringIO()
    pickle.dump(spec, sio)
    sio = StringIO(sio.getvalue())
    spec2 = pickle.load(sio)
    assert spec == spec2


def test_valid():
    for x in valid_stream_spec:
        assert isinstance(x, dict)
        yield check_parsing, x
        yield check_conversions, x
        yield check_pickling, x


def test_invalidi():
    for x in invalid_stream_spec:
        assert isinstance(x, dict)
        yield check_parsing_invalid, x


def test_default_values_1():
    ''' Default default value is middle of range. '''
    spec = StreamSpec.from_yaml(
        {'shape': [3], 'format': 'C', 'range': [0, 1]})
    assert_allclose(spec.get_default_value(), [0.5, 0.5, 0.5])


def test_default_values_2():
    ''' Default default value is middle of range, 
        rounded down for discrete. '''
    spec = StreamSpec.from_yaml(
        {'shape': [3], 'format': 'D', 'range': [0, 1]})
    assert_allclose(spec.get_default_value(), [0, 0, 0])


def test_default_values_3():
    ''' Given default value. '''
    spec = StreamSpec.from_yaml(
        {'shape': [3], 'format': 'D', 'range': [0, 3], 'default': [1, 2, 3]})
    assert_allclose(spec.get_default_value(), [1, 2, 3])


def test_default_values_4():
    ''' Given default value; all the same. '''
    spec = StreamSpec.from_yaml(
        {'shape': [3], 'format': 'D', 'range': [0, 5], 'default': 4})
    assert_allclose(spec.get_default_value(), [4, 4, 4])


def test_default_values_5():
    ''' 2D default values '''
    spec = StreamSpec.from_yaml(
        {'shape': [2, 2], 'format': 'D', 'range': [0, 5], 'default': 4})
    assert_allclose(spec.get_default_value(), [[4, 4], [4, 4]])


def test_compression1():
    spec = StreamSpec.from_yaml(
        {'shape': [100, 100], 'format': 'C', 'range': [0, 5], 'default': 4})

    yaml2 = spec.to_yaml()
    assert yaml2['format'] == 'C'
    assert yaml2['range'] == [0, 5]
    assert yaml2['default'] == 4





