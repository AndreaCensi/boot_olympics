import os

# TODO: make it thread safe


def safe_makedirs(dirname):
    if not os.path.exists(dirname):
        os.makedirs(dirname)


def safe_symlink(source, linkname):
    if os.path.lexists(linkname):
        os.unlink(linkname)
    assert not os.path.lexists(linkname)

    rel_link = os.path.relpath(os.path.realpath(source),
                              os.path.dirname(os.path.realpath(linkname)))
    abs_link = os.path.realpath(source)
    
    if len(rel_link) > len(abs_link):
        link = abs_link
    else:
        link = rel_link
        
    # TODo: check that it was a link
    os.symlink(link, linkname)

#    print('destination: %r' % source)
#    print('   filename: %r' % linkname)
#    print('destination(R): %r' % os.path.realpath(source))
#    print('   filename(R): %r' % os.path.realpath(linkname))
#    print('    rel link: %r ' % rel_link)
#    print('    abs link: %r ' % abs_link)
#    print('      link: %r ' % link)


def mkdirs_thread_safe(dst):
    """Make directories leading to 'dst' if they don't exist yet"""
    if dst == '' or os.path.exists(dst):
        return
    head, _ = os.path.split(dst)
    if os.sep == ':' and not ':' in head:
        head = head + ':'
    mkdirs_thread_safe(head)
    try:
        os.mkdir(dst, 0777)
    except OSError as err:
        if err.errno != 17: #file exists 
            raise


def make_sure_dir_exists(filename):
    ''' Makes sure that the path to file exists, but creating directories. '''
    dirname = os.path.dirname(filename)
    # dir == '' for current dir
    if dirname != '' and not os.path.exists(dirname):
        mkdirs_thread_safe(dirname)


