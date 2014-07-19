from blocks.library.simple.with_queue import WithQueue


__all__ = ['Delay']


class Delay(WithQueue):

    def __init__(self, delay):
        WithQueue.__init__(self)
        self.delay = delay


    def put_noblock(self, value):
        t, x = value
        t2 = t + self.delay
        self.append((t2, x))
