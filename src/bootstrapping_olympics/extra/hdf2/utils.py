from bootstrapping_olympics.utils import yaml_load
from contracts import contract, describe_type
import zlib


@contract(index='int', returns='dict')
def load_extra(extra_table, index):
    extra_string = str(extra_table[index])
    extra_string = zlib.decompress(extra_string)
    extra = yaml_load(extra_string)
    if not isinstance(extra, dict):
        msg = ('Expected to deserialize a dict, obtained %r' 
               % describe_type(extra))
        raise Exception(msg)
    return extra
