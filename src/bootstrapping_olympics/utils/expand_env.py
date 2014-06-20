import os

__all__ = ['expand_environment']

def expand_environment(s):
    ''' Expands ~ and ${ENV} in the string. '''
    s = os.path.expandvars(s)
    s = os.path.expanduser(s)
    return s
