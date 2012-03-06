""" 
    These are very "meta" utils for creating nose tests on the fly. 

    Here is an example use: ::

        thinghies = {'banana': 'yellow', 'apple': 'red', 'sky': 'blue'}

        def thinghies_list():
            return thinghies.keys()
        
        def thinghies_args(x):
            return (x, thinghies[x])
        
        def thinghies_attrs(x):
            return dict(thinghy_name='%s' % x, flavor=thinghies[x])
        
        for_all_thinghies = fancy_test_decorator(lister=thinghies_list,
                                                 arguments=thinghies_args,
                                                 attributes=thinghies_attrs)
                                                 
                                           
    And this is the proper test: ::

        @for_all_thinghies
        def check_good_flavor(id_thinghy, flavor):
            print('test for %s %s' % (id_thinghy, flavor))

        
"""
__all__ = ['fancy_test_decorator']

from nose.tools import istest, nottest
import sys
from .. import logger


def add_to_module(function, module_name):
    module = sys.modules[module_name]
    name = function.__name__

    if not 'test' in name:
        raise Exception('No "test" in function name %r' % name)

    if not 'test' in module_name:
        raise Exception('While adding %r in %r: module does not have "test"'
                        ' in it, so nose will not find the test.' %
                        (name, module_name))

    if name in module.__dict__:
        raise Exception('Already created test %r.' % name)

    module.__dict__[name] = function

    logger.debug('Added test %s:%s' % (module.__name__, name))


def add_checker_f(f, x, arguments, attributes, naming):
    @istest
    def caller():
        args = arguments(x)
        f(*args)

    name = 'test_%s_%s' % (f.__name__, naming(x))
    caller.__name__ = name

    if True: # TODO TMP
        for k, v in attributes(x).items():
            caller.__dict__[k] = v

    add_to_module(caller, f.__module__)


# TODO: add debug info function
@nottest
def fancy_test_decorator(lister,
                       arguments=lambda x: x,
                       attributes=lambda x: {'id': str(x)},
                       naming=lambda x: str(x)):
    ''' 
        Creates a fancy decorator for adding checks.
        
        :param lister: a function that should give a list of objects
        :param arguments: from object to arguments
        :param attributes: (optional) set of attributes for the test
        
        Returns a function that can be used as a decorator.
        
    '''

    def for_all_stuff(check):
        for x in lister():
            add_checker_f(check, x, arguments, attributes, naming)

    return for_all_stuff

