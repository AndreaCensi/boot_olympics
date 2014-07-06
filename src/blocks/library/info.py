from contracts import describe_value

from .with_queue import WithQueue


__all__ = ['Info']

class Info(WithQueue):

    def __init__(self, prefix='Info'):
        WithQueue.__init__(self)
        self.prefix = prefix

    def reset(self):
        pass

    def put_noblock(self, value):
        t, ob = value
        self.info('%s: %s %s' % (self.prefix, t, describe_value(ob, 200)))
        self.append(value)

