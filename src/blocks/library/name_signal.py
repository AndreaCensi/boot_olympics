from .instantaneous import Instantaneous


__all__ = ['NameSignal']


class NameSignal(Instantaneous):

    def __init__(self, name):
        Instantaneous.__init__(self)
        self.name = name

    def transform_value(self, value):
        timestamp, ob = value
        return  timestamp, (self.name, ob)

