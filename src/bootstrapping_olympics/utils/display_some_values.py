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
    indices, = np.nonzero(select)
    indices = indices[:to_display]
    s = 'First %s of %s/%s:' % (to_display, how_many, x.size)
    s += '\n value: %s' % x[indices]
    s += '\n lower: %s' % streamels['lower'][indices]
    s += '\n upper: %s' % streamels['upper'][indices]
    s += '\n index: %s' % indices
    return s
