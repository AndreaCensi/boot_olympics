import os

# TODO: make it thread safe

def safe_makedirs(dirname):
    if not os.path.exists(dirname):
        os.makedirs(dirname)


def safe_symlink(source, linkname):
    source = os.path.relpath(source, os.path.dirname(linkname))
    if os.path.lexists(linkname):
        os.unlink(linkname)
    assert not os.path.lexists(linkname)
    
    # TODo: check that it was a link
    os.symlink(source, linkname)
