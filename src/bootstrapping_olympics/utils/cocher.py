''' Making sure that we use good strings for filenames and identifiers. '''

from . import logger
import re

good_identifier_pattern = re.compile(r"[_A-Za-z][_A-Za-z1-9]*")


def is_good_identifier(s):
    ''' Check that the string is a good identifier. This makes it
        easy to use the name in filenames, fields, etc. '''
    return good_identifier_pattern.match(s) is not None


def warn_good_identifier(s):
    ''' Writes a warning if the string does not match the expectations. '''
    if not is_good_identifier(s):
        logger.warn('The string %r is not a valid identifier. '
                    'This might create some annoyances later.' % s)


def check_good_identifier(s):
    ''' Raises ValueError if the string is not a good identifier. '''
    if not is_good_identifier(s):
        raise ValueError('The string %r is not a good identifer.' % s)


def warn_good_filename(s):
    ''' Warns if the string is not a good filename (for all platforms.) '''
    # TODO: write


def check_good_filename(s):
    ''' Raises an Exception if the string is not a good filename 
        (for all platforms.) '''
    # TODO: write
