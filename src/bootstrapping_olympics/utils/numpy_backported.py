
""" assert_allclose was not available until 1.5 """

try:
    from numpy.testing.utils import assert_allclose #@UnusedImport
except ImportError:
    import numpy as np

    def assert_allclose(actual, desired, rtol=1e-7, atol=0,
                        err_msg='', verbose=True):
        ''' Backporting assert_allclose from Numpy 1.5 to 1.4 '''
        from numpy.testing.utils import assert_array_compare #@UnresolvedImport

        def compare(x, y):
            return np.allclose(x, y, rtol=rtol, atol=atol)

        actual, desired = np.asanyarray(actual), np.asanyarray(desired)
        header = 'Not equal to tolerance rtol=%g, atol=%g' % (rtol, atol)
        assert_array_compare(compare, actual, desired, err_msg=str(err_msg),
                             verbose=verbose, header=header)

