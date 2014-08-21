from contextlib import contextmanager
import os

__version__ = '1.2'

__all__ = [
        'safe_write',
        'safe_write_tmp_filename',
]


@contextmanager
def safe_write(filename, mode='wb', suffix_tmp='.tmp', suffix_old='.old'):
    """
        Also makes sure the directory is created.
    """
    with safe_write_tmp_filename(filename, suffix_tmp=suffix_tmp, suffix_old=suffix_old) as filename_new:
        with open(filename_new, mode) as f:
            yield f


@contextmanager
def safe_write_tmp_filename(filename,  suffix_tmp='.tmp', suffix_old='.old'):
    """
        This yields a temporary filename, not a handle.
    """
    
    dirname = os.path.dirname(filename)
    if not os.path.exists(dirname):
        if dirname != '':
            try:
                os.makedirs(dirname)
            except:
                if os.path.exists(dirname):
                    pass # race condition
                else:
                    raise

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
        yield filename_new
    except:
        #print('Error while cretating %s ' % filename_new)
        if os.path.exists(filename_new):
            try:
                os.unlink(filename_new)
            except:
                pass
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
    
