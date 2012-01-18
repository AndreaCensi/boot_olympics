''' Fast routines to yaml reading and writing. '''

from . import contract, logger
from yaml import load, dump

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    logger.debug('Could not load C YAML reader. '
                 'I can continue but everything will be slow.')
    # TODO: write error
    from yaml import Loader, Dumper


@contract(yaml_string='str')
def yaml_load(yaml_string):
    return load(yaml_string, Loader=Loader)


@contract(returns='str')
def yaml_dump(ob):
    return dump(ob, Dumper=Dumper)


@contract(returns='str')
def yaml_dump_inline(ob):
    ''' Writes in the inline style. E.g. "{a: 1}" instead of 'a: 1'. '''
    return dump(ob, Dumper=Dumper, default_flow_style=True)
