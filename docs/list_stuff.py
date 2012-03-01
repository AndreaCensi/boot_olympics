#!/usr/bin/env python
from conf_tools import import_name, logger
from optparse import OptionParser
import inspect
import sys
from bootstrapping_olympics.utils import wrap_script_entry_point

def main(argv):
    usage = ""
    parser = OptionParser(usage=usage)
    parser.disable_interspersed_args()
    parser.add_option("-i", dest="super",
                      type="string",
                      help='Superclass')
    (options, args) = parser.parse_args(argv)

    if len(args) > 1:
        msg = 'Only one extra arg'
        raise Exception(msg)

    module_name = args[0]
    module = import_name(module_name)

    items = list(module.__dict__.items())

    if options.super is not None:
        options.super = import_name(options.super)

    def same_class(name, object):
        if options.super is None:
            return True
        if object == options.super:
            return False # getmro() returns also the class itself
        try:
            bases = inspect.getmro(object)
            return options.super in bases
        except Exception as e: # XXX 
            return False
        return True


    filters = [same_class]
    for f in filters:
        items = [x for x in items if f(*x)]

    items = sorted(items, key=lambda x: x[0])

    s = """
.. py:module:: %s

.. autosummary::  
   :toctree: api
   
""" % module_name

    for name, ob in items:
        s += ('   %s\n' % name)
    s += '\n\n'

    print(s)

if __name__ == '__main__':
    wrap_script_entry_point(main, logger)

