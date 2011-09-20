from . import logger
from glob import glob
from os.path import splitext, basename
import os
#import pickle
import time
import cPickle as pickle

# XXX: remove this junk
PRINT_STATS = False
def print_stats(method, key, length, duration):
    print("stats: %10s  %8d bytes  %.2fs %s" % (method, length, duration, key))

__all__ = ['StorageFilesystem']

class StorageFilesystem:
    checked_existence = False
    
    def __init__(self, basepath):
        self.basepath = basepath
    
    def get(self, key):
        if not self.exists(key):
            raise Exception('Could not find key %r.' % key)
        filename = self.filename_for_key(key)
        try:
            start = time.time()
            with open(filename, 'rb') as f:
                state = pickle.load(f)
            
            duration = time.time() - start
            if PRINT_STATS:
                length = 0
                print_stats('get    ', key, length, duration)
            return state
        except Exception as e:
            msg = "Could not unpickle file %r: %s" % (filename, e)
            logger.error(msg)
            raise 
        
    def set(self, key, value): #@ReservedAssignment
        if not StorageFilesystem.checked_existence:
            StorageFilesystem.checked_existence = True
            if not os.path.exists(self.basepath):
                os.makedirs(self.basepath)
            
        filename = self.filename_for_key(key)
        filename_new = filename + '.tmp'
        filename_old = filename + '.old'
        if os.path.exists(filename_new):
            # print('Warning; tmp file %r exists (write not succeeded)' 
            # % filename_new)
            os.unlink(filename_new)

        if os.path.exists(filename_old):
            #print('Warning; tmp file %r exists (write not succeeded)' 
            # % filename_old)
            os.unlink(filename_old)

        start = time.time()
        try:
            with open(filename_new, 'wb') as f:
                pickle.dump(value, f, pickle.HIGHEST_PROTOCOL)
        except Exception as e:
            msg = ('Cannot set key %s: cannot pickle object '
                    'of class %s: %s' % (key, value.__class__.__name__, e))
            logger.error(msg)
            raise
        
        if os.path.exists(filename):
            # if we have an old version
            os.rename(filename, filename_old)
            os.rename(filename_new, filename)
            os.unlink(filename_old)
        else:
            # no previous file, just rename
            os.rename(filename_new, filename)
        
        assert os.path.exists(filename)
        assert not os.path.exists(filename_new)
        assert not os.path.exists(filename_old)
            
        duration = time.time() - start
        if PRINT_STATS: 
            length = 0
            print_stats('    set', key, length, duration)
    
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
        
