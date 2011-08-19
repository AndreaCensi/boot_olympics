import os, time 
import cPickle as pickle
from glob import glob
from os.path import  splitext, basename
from StringIO import StringIO 


PRINT_STATS = True
def print_stats(method, key, length, duration):
    print("stats: %10s  %8d bytes  %.2fs %s" % (method, length, duration, key))


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
            file = open(filename, 'rb')
            content = file.read()
            file.close()
            # print "R %s len %d" % (key, len(content))
            sio = StringIO(content)
            state = pickle.load(sio)
            
            duration = time.time() - start
            if PRINT_STATS:
                length = len(content)
                print_stats('get    ', key, length, duration)
            return state
        except Exception as e:
            msg = "Could not unpickle file %r: %s" % (filename, e)
            raise Exception(msg) 
        
    def set(self, key, value):
        if not StorageFilesystem.checked_existence:
            StorageFilesystem.checked_existence = True
            if not os.path.exists(self.basepath):
                os.makedirs(self.basepath)
            
        filename = self.filename_for_key(key)
        start = time.time()
        
        sio = StringIO()
        try:
            pickle.dump(value, sio, pickle.HIGHEST_PROTOCOL)
        except Exception as e:
            raise Exception('Cannot set key %s: cannot pickle object '
                    'of class %s: %s' % (key, value.__class__.__name__, e))
        
        content = sio.getvalue()
        with open(filename, 'wb') as f:
            f.write(content)

        duration = time.time() - start
        if PRINT_STATS:
            length = len(content)
            print_stats('    set', key, length, duration)
    
    def delete(self, key):
        filename = self.filename_for_key(key)
        assert os.path.exists(filename), \
            'I expected path %s to exist before deleting' % filename
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
        
