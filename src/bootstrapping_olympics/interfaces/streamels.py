from . import contract, np
from contracts import check
from bootstrapping_olympics.utils import show_differences


class ValueFormats:
    Continuous = 'C' # range is [min, max]
    Discrete = 'D' # finite number of elements 
    Invalid = 'I' # invalid/not used # TODO: tests for invalid values
    valid = [Continuous, Discrete, Invalid]

streamel_dtype = [('kind', 'S1'), # 'I','D','C'
                  ('lower', 'float'),
                  ('upper', 'float'),
                  ('default', 'float')]


@contract(streamels='array')
def check_valid_streamels(streamels):
    ''' Raises a ValueError if the data in the structure is not coherent. '''
    assert streamels.dtype == np.dtype(streamel_dtype)

    # XXX: how about invalid values?
    def check(which, msg, a, b):
        if not np.all(which):
            error = show_differences(a, b, is_failure=(~which), condition=msg)
            raise ValueError(error)

    not_discrete = streamels['kind'] != ValueFormats.Discrete
    invalid = streamels['kind'] == ValueFormats.Invalid

    check(np.logical_or(invalid, streamels['lower'] < streamels['upper']),
          'lower<upper', streamels['lower'], streamels['upper'])
    check(np.logical_or(invalid, streamels['default'] <= streamels['upper']),
          'default<=upper', streamels['default'], streamels['upper'])
    check(np.logical_or(invalid, streamels['lower'] <= streamels['default']),
          'default<=upper', streamels['lower'], streamels['default'])

    check(np.logical_or(not_discrete,
                    np.round(streamels['default']) == streamels['default']),
                        'default is discrete',
                        np.round(streamels['default']),
                        streamels['default'])

