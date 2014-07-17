from contracts import describe_value


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

def check_timed(value):
    if not isinstance(value, tuple) or not len(value) == 2:
        msg = 'Value is not a timed value: %s' % describe_value(value)
        raise ValueError(msg)

def check_timed_named(value):
    check_timed(value)
    t, what = value  # @UnusedVariable
    if (not isinstance(what, tuple) or not len(what) == 2 or not isinstance(what[0],str)):
        msg = 'Value is not a timed/named value: %s' % describe_value(value)
        raise ValueError(msg)

