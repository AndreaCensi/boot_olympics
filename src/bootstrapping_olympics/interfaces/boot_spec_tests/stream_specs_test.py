from .stream_specs import valid_stream_spec, invalid_stream_spec
from ...interfaces import StreamSpec
from numpy.testing.utils import assert_raises
from contracts import ContractNotRespected


def check_parsing(x):
    StreamSpec.from_yaml(x)    

def check_parsing_invalid(x):
    assert_raises((ValueError, ContractNotRespected), StreamSpec.from_yaml, x)    


def check_conversions(x):
    spec = StreamSpec.from_yaml(x)
    assert isinstance(spec, StreamSpec)
    y = spec.to_yaml()
    assert isinstance(y, dict)
    spec2 = spec.from_yaml(y)
    assert spec == spec2
    
def test_valid():
    for x in valid_stream_spec:
        assert isinstance(x, dict)
        yield check_parsing, x
        yield check_conversions, x

def test_invalidi():
    for x in invalid_stream_spec:
        assert isinstance(x, dict)
        yield check_parsing_invalid, x
