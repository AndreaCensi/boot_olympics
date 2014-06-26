from abc import abstractmethod

from blocks import WithQueue


__all__ = ['Instantaneous']

class Instantaneous(WithQueue):

    @abstractmethod
    def transform_value(self, value):
        pass

    def put(self, value, block=False, timeout=None):  # @UnusedVariable
        value2 = self.transform_value(value)
        self.queue.append(value2)

