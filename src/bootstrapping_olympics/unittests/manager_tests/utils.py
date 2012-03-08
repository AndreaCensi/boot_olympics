from contextlib import contextmanager
from shutil import rmtree
from tempfile import mkdtemp


@contextmanager
def create_tmp_dir():
    root = mkdtemp()
    try:
        yield root
    finally:
        rmtree(root)
