from .with_queue import WithQueue
from contracts import describe_value



__all__ = ['Info']

class Info(WithQueue):

    def __init__(self, prefix='Info'):
        WithQueue.__init__(self)
        self.prefix = prefix

    def __repr__(self):
        return 'Info(%r)' % self.prefix

    def put_noblock(self, value):
        from blocks.library.timed.checks import check_timed

        check_timed(value, self)
        t, ob = value
        self.info('%s: t = %s %s' % (self.prefix, t, describe_value(ob, 200)))
        
        self.append(value)

