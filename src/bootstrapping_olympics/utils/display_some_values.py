import numpy as np

__all__ = ['display_some', 'display_some_extended']


def display_some(x, select, max_to_display=4):
    ''' Displays some of the elements in x (array) given
        by select (array of bool). '''
    how_many = np.sum(select)
    to_display = min(how_many, max_to_display)
    s = 'First %s of %s/%s: %s' % (to_display, how_many, x.size,
                                   x[select][:to_display])
    return s


def display_some_extended(x, streamels, select, max_to_display=4):
    ''' Displays some of the elements in x (array) given
        by select (array of bool). '''
    how_many = np.sum(select)
    to_display = min(how_many, max_to_display)
    values = x[select][:to_display]
    s = 'First %s of %s/%s: %s' % (to_display, how_many,
                                   x.size, values)
    s += '\n lower: %s' % streamels['lower'][select][:to_display]
    s += '\n upper: %s' % streamels['upper'][select][:to_display]
    return s
