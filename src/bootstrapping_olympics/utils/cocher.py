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


def strange_chars_for_filename(s):
    """ Returns None if no strange chars are detected, or a string
        with an explanation. """
    forbidden = [':', ',', '"', "'", ' ']
    for f in forbidden:
        if f in s:
            return 'Forbidden character %r found.' % f
    return None


def warn_good_filename(s):
    ''' Warns if the string is not a good filename (for all platforms.) '''
    strange = strange_chars_for_filename(s)
    if strange:
        msg = 'Consider changing the filename %r: %s' % (s, strange)
        logger.warn(msg)


def check_good_filename(s):
    ''' Raises an Exception if the string is not a good filename 
        (for all platforms.) '''
    strange = strange_chars_for_filename(s)
    if strange:
        msg = 'Consider changing the filename %r: %s' % (s, strange)
        raise ValueError(msg)
