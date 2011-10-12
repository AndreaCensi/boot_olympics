import datetime


def isodate():
    """ E.g., '2011-10-06-22:54' """
    now = datetime.datetime.now()
    date = now.isoformat('-')[:16]
    return date


def isodate_with_secs():
    """ E.g., '2011-10-06-22:54:33' """
    now = datetime.datetime.now()
    date = now.isoformat('-')[:19]
    return date


def unique_timestamp_string():
    now = datetime.datetime.now()
    s = now.isoformat()
    return s.replace('-', '').replace(':', '').replace('T', '_').replace('.', '_')
