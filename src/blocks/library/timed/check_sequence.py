from .checks import check_timed_format
from blocks.interface import SimpleBlackBoxT, SimpleBlackBoxTN
from blocks.library import WithQueue
from blocks.library.timed.checks import check_timed_named
from collections import defaultdict


__all__ = [
    'CheckSequence',
    'CheckSequenceTN',
]

    
class CheckSequence(SimpleBlackBoxT, WithQueue):
    """ Checks that the timestamps are ordered (<=). """

    def __init__(self):
        WithQueue.__init__(self)

    def reset(self):
        WithQueue.reset(self)
        self.last_t = None

    def put_noblock(self, value):
        check_timed_format(self, value)
        t, _ = value

        # self.info('found timestamp %s' % t)

        if self.last_t is not None:
            if t < self.last_t:
                msg = 'Found timestamps in wrong order.'
                msg += '\n last: %s  cur: %s' % (self.last_t, t)
                raise ValueError(msg)
        self.append(value)




class CheckSequenceTN(SimpleBlackBoxTN, WithQueue):
    """ Checks that the timestamps are ordered 
        Each signal should be strictly incremental. """

    def __init__(self):
        WithQueue.__init__(self)

    def reset(self):
        WithQueue.reset(self)
        self.last = defaultdict(lambda: None)

    def put_noblock(self, value):
        check_timed_named(value)
        t, (s, _) = value

        last = self.last[s]
        if last is not None:
            if not ( t > last):
                msg = 'Found timestamps in wrong order for signal %r.' % s
                msg += '\n last: %.5f cur: %.5f' % (last, t)
                raise ValueError(msg)
            
            
        self.last[s] = t

        self.append(value)