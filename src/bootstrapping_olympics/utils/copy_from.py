
def copy_from(a, b):
    ''' Copies all fields of a from b to a. '''
    for name in a.dtype.names:
        a[name].flat = b[name].flat
