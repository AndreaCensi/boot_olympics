import datetime


def isodate():
    now = datetime.datetime.now()
    date = now.isoformat('-')[:16]
    return date

