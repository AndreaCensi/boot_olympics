from abc import abstractmethod

from blocks import NotReady, Finished

from .with_queue import WithQueue


__all__ = [
    'Instantaneous',
    'InstantaneousF',
    'WrapTimedNamed',
]

class Instantaneous(WithQueue):

    def __init__(self):
        WithQueue.__init__(self)

    @abstractmethod
    def transform_value(self, value):
        pass

    def put_noblock(self, value):
        value2 = self.transform_value(value)
        self.append(value2)

    def reset(self):
        pass


class InstantaneousF(Instantaneous):
    """ Instantaneous transform with given function. """

    def __init__(self, func):
        WithQueue.__init__(self)
        self.func = func

    def transform_value(self, value):
        return self.func(value)


class WrapTimedNamed(WithQueue):

    def __init__(self, inside):
        WithQueue.__init__(self)
        self.inside = inside
        self.last_t = None
        self.last_name = None
        
    def end_input(self):
        self.inside.end_input()
        if self.last_t is not None:
            self._pump()

    def put_noblock(self, value):
        self.last_t, (self.last_name, ob) = value
        self.inside.put(ob)
        self._pump()

    def _pump(self):
        while True:
            try:
                ob2 = self.inside.get(block=False)
                value2 = self.last_t, (self.last_name, ob2)
                self.append(value2)
            except NotReady:
                break
            except Finished:  # XXX
                self._finished = True


        
