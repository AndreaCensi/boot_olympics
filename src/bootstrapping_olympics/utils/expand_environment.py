import os

def expand_environment(s):
    ''' Expands ~ and ${ENV} in the string. '''
    s = os.path.expandvars(s)
    s = os.path.expanduser(s)
    return s
