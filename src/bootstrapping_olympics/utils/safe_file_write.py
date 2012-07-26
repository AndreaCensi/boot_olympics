from contextlib import contextmanager
import os

__version__ = '1.1'


@contextmanager
def safe_write(filename, mode='wb', suffix_tmp='.tmp', suffix_old='.old'):
    """
        Also makes sure the directory is created.
    """
    
    dirname = os.path.dirname(filename)
    if not os.path.exists(dirname):
        os.makedirs(dirname)

    filename_new = filename + suffix_tmp
    filename_old = filename + suffix_old
    if os.path.exists(filename_new):
        # print('Warning; tmp file %r exists (write not succeeded)' 
        # % filename_new)
        os.unlink(filename_new)

    if os.path.exists(filename_old):
        #print('Warning; tmp file %r exists (write not succeeded)' 
        # % filename_old)
        os.unlink(filename_old)

    # TODO: make all of this robust to race conditions
    try:
        with open(filename_new, mode) as f:
            yield f
    except:
        if os.path.exists(filename_new):
            os.unlink(filename_new)
        raise

    if os.path.exists(filename):
        # if we have an old version
        os.rename(filename, filename_old)
        os.rename(filename_new, filename)
        os.unlink(filename_old)
    else:
        # no previous file, just rename
        os.rename(filename_new, filename)

    if False:
        # TODO FIXME make sure this works in concurrent 
        assert os.path.exists(filename)
        assert not os.path.exists(filename_new)
        assert not os.path.exists(filename_old)
        
