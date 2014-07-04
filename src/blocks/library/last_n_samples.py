from contracts import contract

from .with_queue import WithQueue


__all__ = ['LastNSamples']


class LastNSamples(WithQueue):
    """ Collects N samples. Output is a list of length N. """

    @contract(N='int')
    def __init__(self, N):
        WithQueue.__init__(self)
        self.N = N

    def reset(self):
        self.last = []

    def put_noblock(self, value):
        self.last.append(value)
        if len(self.last) >= self.N:
            out = self.last[:self.N]
            self.append(out)
            self.last = self.last[1:]

