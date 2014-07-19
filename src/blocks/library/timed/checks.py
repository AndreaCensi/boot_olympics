from contracts import describe_value
from contracts.utils import format_obs, indent


__all__ = [
  'check_timed_format',
  'check_timed',
  'check_timed_named',
]


def check_timed_format(block, value):
    if not isinstance(value, tuple) or not len(value) == 2:
        msg = 'Value is not a timed value: %s' % describe_value(value)
        block.error(msg)
        raise ValueError(msg)

def check_timed(value, block=None):
    if not isinstance(value, tuple) or not len(value) == 2 or not isinstance(value[0], float):
        msg = 'Value is not a timed value.'
        msg += '\n' + indent(format_obs(dict(block=block,value=value)), ' ')
        if block is not None:
            block.error(msg)
        raise ValueError(msg)

def check_timed_named(value, block=None):
    check_timed(value)
    t, what = value  # @UnusedVariable
    if (not isinstance(what, tuple) or not len(what) == 2 or not isinstance(what[0],str)):
        msg = 'Value is not a timed/named value'        
        msg += '\n' + indent(format_obs(dict(block=block,value=value)), ' ')
        
        if block is not None:
            block.error(msg)
        raise ValueError(msg)

