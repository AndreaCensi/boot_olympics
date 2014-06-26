from blocks.with_queue import WithQueue


__all__ = ['NameSignal']


class NameSignal(WithQueue):

    def __init__(self, name):
        WithQueue.__init__(self)
        self.name = name

    def put(self, value, block=False, timeout=None):
        timestamp, ob = value
        x = timestamp, (self.name, ob)
        self.append(x)
