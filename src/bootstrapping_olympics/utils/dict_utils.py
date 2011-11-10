
        
def check_contained(key, D, name='key'):
    ''' Raises a ValueError if key is not in the dictionary-like object D. '''
    if not key in D:
        msg = 'No %s %r found among %s.' % (name, key, D.keys())
        # TODO: pretty print
        # TODO: only print a limited number
        # TODO: add suggestions
        raise ValueError(msg)
