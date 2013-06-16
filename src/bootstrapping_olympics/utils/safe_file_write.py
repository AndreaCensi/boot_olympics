from contextlib import contextmanager
import os
import warnings

__version__ = '1.1'

__all__ = ['safe_write']


@contextmanager
def safe_write(filename, mode='wb', suffix_tmp='.tmp', suffix_old='.old'):
    """
        Also makes sure the directory is created.
    """
    
    dirname = os.path.dirname(filename)
    if not os.path.exists(dirname):
        if dirname != '':
            os.makedirs(dirname)

    # TODO: use tmpfile 
    filename_new = filename + suffix_tmp
    filename_old = filename + suffix_old
    if os.path.exists(filename_new):
        # print('Warning; tmp file %r exists (write not succeeded)' 
        # % filename_new)
        os.unlink(filename_new)

    if os.path.exists(filename_old):
        # print('Warning; tmp file %r exists (write not succeeded)' 
        # % filename_old)
        os.unlink(filename_old)

    try:
        with open(filename_new, mode) as f:
            yield f
    except:
        if os.path.exists(filename_new):
            os.unlink(filename_new)
        raise

    try:
        # TODO: make all of this robust to race conditions
        # for now, we just make sure that the file exists in the 
        # end
        if os.path.exists(filename):
            # if we have an old version
            os.rename(filename, filename_old)
            os.rename(filename_new, filename)
            os.unlink(filename_old)
        else:
            # no previous file, just rename
            os.rename(filename_new, filename)
    except OSError:
        # for example, somebody already 
        pass
    
    if False:
        # TODO FIXME make sure this works in concurrent 
        assert os.path.exists(filename)
        assert not os.path.exists(filename_new)
        assert not os.path.exists(filename_old)
    
    warnings.warn('this was a race condition')
#         
#     if not os.path.exists(filename):
#         msg = "Sorry, filename %r does not exists. Race conditions. It's a bug." % filename
#         raise Exception(msg)
#         
