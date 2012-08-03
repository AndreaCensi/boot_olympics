import cPickle as pickle
from . import safe_write, logger
from compmake.utils.describe import describe_type


def safe_pickle_dump(value, filename, protocol=pickle.HIGHEST_PROTOCOL):
    # TODO: add debug
    with safe_write(filename) as f:
        try:
            pickle.dump(value, f, protocol)
        except Exception:
            # TODO: add debug check
            msg = 'Cannot pickle object of class %s' % describe_type(value)
            logger.error(msg)
            raise 
    
def safe_pickle_load(filename):
    # TODO: add debug check 
    with open(filename, 'rb') as f:
        return pickle.load(f)
