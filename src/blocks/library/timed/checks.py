from contracts import describe_value


__all__ = [
  'check_timed_format',
]


def check_timed_format(block, value):
    if not isinstance(value, tuple) or not len(value) == 2:
        msg = 'Value is not a timed value: %s' % describe_value(value)
        block.error(msg)
        raise ValueError(msg)
