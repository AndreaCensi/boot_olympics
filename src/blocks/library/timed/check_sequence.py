from blocks.library import WithQueue

from .checks import check_timed_format


__all__ = ['CheckSequence']

    
class CheckSequence(WithQueue):
    """ Checks that the timestamps are ordered. """

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
