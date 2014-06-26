from contracts import contract

from blocks import NotReady, Finished
from blocks.simple_black_box import SimpleBlackBox
from blocks.with_queue import WithQueue


__all__ = ['Split']


class Split(WithQueue):
    """ Duplicates one input to two subsystems and merges their output. """

    @contract(a=SimpleBlackBox, b=SimpleBlackBox)
    def __init__(self, a, b, name1, name2):
        WithQueue.__init__(self)
        self.a = a
        self.b = b
        self.a_finished = False
        self.b_finished = False
        self.log_add_child(name1, a)
        self.log_add_child(name2, b)

    def put(self, value, block=False, timeout=None):
        # TODO: does not consider Full as a special case
        self.a.put(value, block, timeout)
        self.b.put(value, block, timeout)

        self._pump()
    def _pump(self):
        new_obs = []
        if not self.a_finished:
            while True:
                try:
                    x = self.a.get(block=False)
                    new_obs.append(x)
                except NotReady:
                    self.info('First is not ready')
                    break
                except Finished:
                    self.info('Now first is finished')
                    self.a_finished = True
                    break
        if not self.b_finished:
            while True:
                try:
                    x = self.b.get(block=False)
                    new_obs.append(x)
                except NotReady:
                    self.info('First is not ready')
                    break
                except Finished:
                    self.info('Now second is finished')
                    self.b_finished = True
                    break
        # Sort by timestamp
        s = sorted(new_obs, key=lambda x: x[0])
        for x in s:
            self.append(x)

        if self.a_finished and self.b_finished:
            self._finished = True

    def end_input(self):
        self.a.end_input()
        self.b.end_input()
        self._pump()