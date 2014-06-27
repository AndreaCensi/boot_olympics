from .with_queue import WithQueue


__all__ = ['Collect']


class Collect(WithQueue):
    """ Collects all the signals with the same timestamp. """

    def __init__(self):
        WithQueue.__init__(self)
        # XXX
        self.last = {}
        self.last_t = None

    def put_noblock(self, value):
        t, (s, x) = value

        if self.last_t is not None:
            if t > self.last_t:
                self._flush()

        assert not s in self.last
        self.last[s] = x

        self.last_t = t

        self.info('cur: %s %s ' % (self.last_t, self.last))

    def _flush(self):
        if self.last:
            self.append((self.last_t, self.last))
            self.last = {}

    def end_input(self):
        if self.last:
            self._flush()
            self.last = {}
        self._finished = True

