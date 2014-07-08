from .with_queue import WithQueue
from contracts import contract
from blocks.utils import check_reset


__all__ = ['Collect', 'CollectSignals']


class Collect(WithQueue):
    """ 
        Collects all the signals with the same timestamp. 
    
        Warning: not instantaneous -- see CollectSignals for the instantaneous
        version.
    """

    def __init__(self):
        WithQueue.__init__(self)

    def __str__(self):
        return 'Collect()'

    def reset(self):
        WithQueue.reset(self)
        self.last = {}
        self.last_t = None

    def put_noblock(self, value):
        check_reset(self, 'last')
        t, (s, x) = value
        self.info('seeing %s %s' % (t, s))

        if self.last_t is not None:
            if t > self.last_t:
                self._flush()

        assert not s in self.last
        self.last[s] = x

        self.last_t = t

        # self.info('cur: %s %s ' % (self.last_t, self.last))

    def _flush(self):
        if self.last:
            self.info('sending %s %s' % (self.last_t, set(self.last)))
            self.append((self.last_t, self.last))
            self.last = {}

    def end_input(self):
        self._flush()
        self._finished = True





class CollectSignals(WithQueue):
    """ 
        Collects all the signals with the same timestamp.
        
        Note that Collect() waits for the next timestamp t_{k+1} to
        release the signals at t_{k}. 
        
        CollectSignals is given the set of signals to wait for 
        so it will trigger at t_{k}.
    
    """

    @contract(signals='set(str)')
    def __init__(self, signals):
        WithQueue.__init__(self)
        self.signals = signals

    def __str__(self):
        return 'CollectSignals()'

    def reset(self):
        WithQueue.reset(self)
        self.last = {}
        self.last_t = None

    def put_noblock(self, value):
        check_reset(self, 'last')
        t, (s, x) = value
#         self.info('seeing %s %s' % (t, s))

        if self.last_t is not None:
            if t > self.last_t and self.last:
                msg = 'Flushing incomplete'
                self.warn(msg)
                self._flush()

        assert not s in self.last
        self.last[s] = x

        self.last_t = t

        if set(self.last.keys()) == self.signals:
            self._flush()

        # self.info('cur: %s %s ' % (self.last_t, self.last))

    def _flush(self):
        if self.last:
#             self.info('sending %s %s' % (self.last_t, set(self.last)))
            self.append((self.last_t, self.last))
            self.last = {}

    def end_input(self):
        self._flush()
        self._finished = True

