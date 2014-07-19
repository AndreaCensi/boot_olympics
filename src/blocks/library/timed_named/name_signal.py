from blocks.library.simple import Instantaneous
from blocks.library.timed.checks import check_timed_named, check_timed


__all__ = ['NameSignal']


class NameSignal(Instantaneous):

    def __init__(self, name):
        Instantaneous.__init__(self)
        self.name = name

    def transform_value(self, value):
        check_timed(value)
        timestamp, ob = value
        res = timestamp, (self.name, ob)
        check_timed_named(res)
        return res


class DropNameSignal(Instantaneous):

    def __init__(self, name):
        Instantaneous.__init__(self)
        self.name = name

    def transform_value(self, value):
        check_timed_named(value)
        timestamp, (name, ob) = value
        if name != self.name:
            msg = 'Found signal %r rather than %r.' % (name, self.name)
            raise ValueError(msg)
        res = (timestamp, ob)
        check_timed(res)
        return res 

