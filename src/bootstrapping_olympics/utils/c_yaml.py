''' Fast routines to yaml reading and writing. '''

from . import contract, logger
from yaml import load, dump
from contracts import describe_type
from types import NoneType
from pprint import pprint

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    logger.warn('Could not load C YAML reader. '
                 'I can continue but everything will be slow.')
    # TODO: write error
    from yaml import Loader, Dumper


@contract(yaml_string='str')
def yaml_load(yaml_string):
    try:
        return load(yaml_string, Loader=Loader)
    except:
        logger.error('Could not deserialize YAML')
        dump_emergency_string(yaml_string)
        raise


def dump_emergency_string(s):
    emergency = '/home/andrea/yaml_load.yaml' # XXX
    with open(emergency, 'w') as f:
        f.write(s)
    logger.error('String written to %r.' % emergency)


@contract(returns='str')
def yaml_dump(ob):
    check_pure_structure(ob)
    string = dump(ob, Dumper=Dumper)
    if '!python/object' in string:
        dump_emergency_string(string)
        msg = 'Invalid YAML produced'
        raise ValueError(msg)
    return string


@contract(returns='str')
def yaml_dump_inline(ob):
    ''' Writes in the inline style. E.g. "{a: 1}" instead of 'a: 1'. '''
    check_pure_structure(ob)
    return dump(ob, Dumper=Dumper, default_flow_style=True)


def check_pure_structure(s):
    try:
        check_pure_structure_fast(s)
    except ValueError:
        pprint(s)
        check_pure_structure_detailed(s)


def check_pure_structure_fast(s):
    if isinstance(s, (str, int, float, NoneType)):
        return
    elif isinstance(s, list):
        for x in s:
            check_pure_structure(x)
    elif isinstance(s, dict):
        for k, v in s.items():
            check_pure_structure(k)
            check_pure_structure(v)
    else:
        msg = 'Invalid type for YAML serialization %s' % describe_type(s)
        raise ValueError(msg)


def check_pure_structure_detailed(s, context=None):
    if context is None:
        context = []
    if isinstance(s, (str, int, float, NoneType)):
        return
    elif isinstance(s, list):
        for i, x in enumerate(s):
            context.append('- %dth element of array' % i)
            check_pure_structure_detailed(x, context)
            context.pop()
    elif isinstance(s, dict):
        for k, v in s.items():
            context.append('- key of dictionary')
            check_pure_structure_detailed(k, context)
            context.pop()

            context.append('- value of %r' % k)
            check_pure_structure_detailed(v, context)
            context.pop()
    else:
        msg = ('Invalid type %s for YAML serialization.\n%s' %
               (describe_type(s), "\n".join(context)))
        raise ValueError(msg)




