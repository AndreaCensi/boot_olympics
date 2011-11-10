from . import contract, np
from contracts import check 


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
    def check(which, msg):
        if not np.all(which):
            raise ValueError(msg)
    not_discrete = streamels['kind'] != ValueFormats.Discrete
    invalid = streamels['kind'] == ValueFormats.Invalid
    
    check(np.logical_or(invalid, streamels['lower'] < streamels['upper']),
          'lower<upper')
    check(np.logical_or(invalid, streamels['default'] <= streamels['upper']),
          'default<=upper')
    check(np.logical_or(invalid, streamels['lower'] <= streamels['default']),
          'default<=upper')
    
    check(np.logical_or(not_discrete,
                        np.round(streamels['default']) == streamels['default']),
                        'default is discrete')
    
