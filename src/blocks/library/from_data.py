from contracts import contract

from blocks import Finished, Source


__all__ = ['FromData']

class FromData(Source):

    @contract(data='list(tuple(float, *))')
    def __init__(self, data):
        self.data = data

    def __str__(self):
        return 'FromData(%d)' % len(self.data)

    def reset(self):
        self.k = 0

    def get(self, block=True, timeout=None):  # @UnusedVariable
        if self.k >= len(self.data):
            msg = 'finished length %d' % self.k
            self.info(msg)
            raise Finished(msg)
        res = self.data[self.k]
        self.k += 1
        return res
