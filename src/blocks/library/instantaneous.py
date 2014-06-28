from abc import abstractmethod

from .with_queue import WithQueue


__all__ = ['Instantaneous']

class Instantaneous(WithQueue):

    def __init__(self):
        WithQueue.__init__(self)

    @abstractmethod
    def transform_value(self, value):
        pass

    def put_noblock(self, value):
        value2 = self.transform_value(value)
        self.append(value2)

