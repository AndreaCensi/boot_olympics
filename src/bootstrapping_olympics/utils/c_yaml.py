''' Fast routines to yaml reading and writing. '''

from contracts import contract#, describe_value
from yaml import load, dump

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper


@contract(yaml_string='str')
def yaml_load(yaml_string):
    return load(yaml_string, Loader=Loader)

@contract(returns='str')
def yaml_dump(ob):
    return dump(ob, Dumper=Dumper)
