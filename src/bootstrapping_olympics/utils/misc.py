from pprint import pformat
from contracts import contract

def check_required(required, params):
    for req in required:
        if not req in params:
            msg = 'Necessary parameter %r not given.' % req
            msg += '\nGiven configuration:\n%s' % pformat(params)
            raise Exception(msg)

def check_no_extra(all_possible, params):
    for p in params:
        if not p in all_possible:
            msg = 'Extra parameter %r given, not in %r.' % (p, all_possible)
            msg += '\nGiven configuration:\n%s' % pformat(params)
            raise Exception(msg)

@contract(params='dict', required='list(str)', optional='list(str)')
def check_parameters(params, required, optional):
    check_required(required, params)
    check_no_extra(required + optional, params)
    
