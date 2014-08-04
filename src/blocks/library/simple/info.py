from .with_queue import WithQueue
from contracts import describe_value

__all__ = [
    'Info',
    'InfoT',
    'InfoTN',
]

class Info(WithQueue):

    def __init__(self, prefix='Info'):
        WithQueue.__init__(self)
        self.prefix = prefix

    def __repr__(self):
        return 'Info(%r)' % self.prefix

    def put_noblock(self, value):
        self.info('%s: %s' % (self.prefix, describe_value(value, 200)))        
        self.append(value)


class InfoT(WithQueue):

    def __init__(self, prefix='Info'):
        WithQueue.__init__(self)
        self.prefix = prefix

    def __repr__(self):
        return 'InfoT(%r)' % self.prefix

    def put_noblock(self, value):
        from blocks.library.timed.checks import check_timed

        check_timed(value, self)
        t, ob = value
        self.info('%s: t = %s %s' % (self.prefix, t, describe_value(ob, 200)))

        self.append(value)


class InfoTN(WithQueue):

    def __init__(self, prefix='Info'):
        WithQueue.__init__(self)
        self.prefix = prefix

    def __repr__(self):
        return 'InfoTN(%r)' % self.prefix

    def put_noblock(self, value):
        from blocks.library.timed.checks import check_timed_named
        check_timed_named(value, self)
        t, (s, ob) = value
        self.info('%s: t = %.5f %s %s' % (self.prefix, t, s, 
                                          describe_value(ob, 200)))

        self.append(value)
