

# A couple of functions for pretty errors
def aslist(x):
    if isinstance(x, dict):
        x = list(x.keys())
    if x:
        return ", ".join(sorted(x))
    else:
        return "<empty>"


def raise_x_not_found(what, x, iterable, exception=ValueError):
    msg = x_not_found(what, x, iterable)
    raise exception(msg)

def x_not_found(what, x, iterable):
    ''' Shortcut for creating pretty error messages. '''
    # TODO: add guess in case of typos
    options = aslist(iterable)
    
    return ('Could not find %s %r. I know %s.' % 
            (what, x, options))
