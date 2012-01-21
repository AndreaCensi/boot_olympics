''' Some utils for checking and comparing arrays. '''
import numpy as np
from contracts import contract
from . import assert_allclose


def assert_allequal_verbose(a, b, **kwargs):
    failures = a != b
    if np.any(failures):
        error = show_differences(a, b, failures, "a==b")
        raise ValueError(error)


def assert_allclose_verbose(a, b, **kwargs):
    try:
        assert_allclose(a, b, **kwargs)
    except AssertionError as e:
        is_failure = a != b
        condition = 'a==b' # XXX
        error = show_differences(a, b, is_failure, condition)
        raise Exception('%s\n\n%s' % (e, error))

#
#def close(a, b, rtol=1.e-5, atol=1.e-8):
#    """
#    Same as allclose() but returns the result element by element.
#    """
#    x = array(a, copy=False, ndmin=1)
#    y = array(b, copy=False, ndmin=1)
#    xinf = isinf(x)
#    if not all(xinf == isinf(y)):
#        return False
#    if not any(xinf):
#        return all(less_equal(absolute(x-y), atol + rtol * absolute(y)))
#    if not all(x[xinf] == y[xinf]):
#        return False
#    x = x[~xinf]
#    y = y[~xinf]
#    return all(less_equal(absolute(x-y), atol + rtol * absolute(y)))


@contract(a='shape(x)',
          b='shape(x)',
          is_failure='shape(x)', #,array(bool)',
          condition='str',
          MAX_N='int,>=1')
def show_differences(a, b, is_failure, condition, MAX_N=4):
    """ Returns a string summarizing the failures. """
    some, = np.nonzero(is_failure.flat)
    num = a.size
    num_fail = len(some)
    perc = 100.0 * num_fail / num
    error = ("In this array, %d/%d (%f%%) of elements do not respect "
             "the condition %s." % (num_fail, num, perc, condition))

    N = min(len(some), MAX_N)
    error += '\nThese are the first %d:' % N
    for i in range(min(len(some), MAX_N)):
        error += ('\n a[%d] = %10s  b[%d] = %10s ' % (i, a.flat[i],
                                                      i, b.flat[i]))
    return error
