from . import logger
from bootstrapping_olympics.utils import warn_long_time
from bootstrapping_olympics.utils.safe_pickle import (safe_pickle_dump,
    safe_pickle_load)
from glob import glob
from os.path import splitext, basename
from pickle import HIGHEST_PROTOCOL
import os


__all__ = ['StorageFilesystem']


class StorageFilesystem:
    checked_existence = False

    def __init__(self, basepath, warn_long_time=1):
        self.basepath = basepath
        self.warn_long_time = warn_long_time

    def get(self, key):
        if not self.exists(key):
            raise Exception('Could not find key %r.' % key)
        
        filename = self.filename_for_key(key)
        try:
            with warn_long_time(self.warn_long_time, 'reading %r' % key):  
                return safe_pickle_load(filename)

        except Exception as e:
            msg = "Could not unpickle file %r: %s" % (filename, e)
            logger.error(msg)
            raise

    def set(self, key, value):  # @ReservedAssignment
        """ Return a dictionary with some statistics """
        if not StorageFilesystem.checked_existence:
            StorageFilesystem.checked_existence = True
            if not os.path.exists(self.basepath):
                os.makedirs(self.basepath)

        # TODO: generalize this
        filename = self.filename_for_key(key)

        with warn_long_time(self.warn_long_time,
                            'dumping %r' % key) as moreinfo:        
            safe_pickle_dump(value, filename, protocol=HIGHEST_PROTOCOL)
            moreinfo['size'] = os.stat(filename).st_size
            
        # TODO: remove this
        stats = {}
        stats['duration'] = 0  # XXX
        stats['clock'] = 0  # XXX
        stats['size'] = os.stat(filename).st_size
        return stats
    
    # TODO: remove this
    def stats_string(self, stats):
        """ Formats the string returned by set() """
        return ("Size %.2fMB written in %.2fs (clock: %.2f)" % 
                (stats['size'] * 0.000001, stats['duration'], stats['clock']))
         
    def delete(self, key):
        filename = self.filename_for_key(key)
        if not os.path.exists(filename):
            msg = 'I expected path %s to exist before deleting' % filename
            raise ValueError(msg)
        os.remove(filename)

    def exists(self, key):
        filename = self.filename_for_key(key)
        return os.path.exists(filename)

    def keys(self, pattern='*'):
        filename = self.filename_for_key(pattern)
        for x in glob(filename):
            b = splitext(basename(x))[0]
            yield StorageFilesystem.filename2key(b)

    dangerous_chars = {
       '/': 'CMSLASH',
       '..': 'CMDOT',
       '~': 'CMHOME'
    }

    @staticmethod
    def key2filename(key):
        '''turns a key into a reasonable filename'''
        for char, replacement in StorageFilesystem.dangerous_chars.items():
            key = key.replace(char, replacement)
        return key

    @staticmethod
    def filename2key(key):
        ''' Undoes key2filename '''
        for char, replacement in StorageFilesystem.dangerous_chars.items():
            key = key.replace(replacement, char)
        return key

    def filename_for_key(self, key):
        """ Returns the pickle storage filename corresponding to the job id """
        return os.path.join(self.basepath,
                            StorageFilesystem.key2filename(key) + '.pickle')

