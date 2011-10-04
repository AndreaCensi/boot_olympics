from bootstrapping_olympics.interfaces import BootSpec
from numpy.testing.utils import assert_raises
from .boot_specs import valid_boot_specs, \
    invalid_boot_specs
from contracts import ContractNotRespected



def check_parsing(x):
    BootSpec.from_yaml(x)    

def check_parsing_invalid(x):
    assert_raises((ValueError, ContractNotRespected), BootSpec.from_yaml, x)    

def check_conversions(x):
    spec = BootSpec.from_yaml(x)
    assert isinstance(spec, BootSpec)
    y = spec.to_yaml()
    assert isinstance(y, dict)
    spec2 = spec.from_yaml(y)
    assert spec == spec2
    
def test_validity_1():
    for x in valid_boot_specs:
        yield check_parsing, x
        yield check_conversions, x
        

def test_validity_2():
    for x in invalid_boot_specs:
        yield check_parsing_invalid, x
