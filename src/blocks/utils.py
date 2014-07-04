from contracts import contract


__all__ = ['check_reset']

@contract(attrname='str')
def check_reset(block, attrname):
    """ 
        checks that reset() has been called on the block by
        making sure that there is the given attribute. 
    """
    if not attrname in block.__dict__:
        msg = 'You forgot to call reset() on this block.\n'
        msg += '\nattribute not present: %r' % attrname
        msg += '\nblock: %s' % block
        raise Exception(msg)
