import os
import yaml
from contracts import contract
from bootstrapping_olympics import logger

__all__ = ['read_hints']

@contract(returns='dict')
def read_hints(filename, hints_filename="boot_log.hints.yaml"):
    """
        Reads a yaml file in the same directory and returns a dictionary.
        Returns {} if not hints file is found.
    """
    dirname = os.path.dirname(filename)
    hints = os.path.join(dirname, hints_filename)
    if not os.path.exists(hints):
        logger.debug("No hints file found %s" % hints)
        return {}

    res = yaml.load(open(hints))
    logger.debug('Hints: %s' % res)
    return res
    
