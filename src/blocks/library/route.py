import warnings

from contracts import contract, describe_value

from blocks import NotReady, Finished

from .with_queue import WithQueue


__all__ = ['Route']


class Route(WithQueue):
    """
        Routes signals among children according to user-defined rules.
         
    """
    @contract(routing='list(tuple(dict, isinstance(SimpleBlackBox), dict))')
    def __init__(self, routing):
        WithQueue.__init__(self)
        self.routing = routing

        self.boxes = [box for _, box, _ in self.routing]

        for i, b in enumerate(self.boxes):
            self.log_add_child('%d' % i, b)

        self.finished = [False for i in range(len(self.boxes))]

    def put_noblock(self, value):

        t, (name, ob) = explode_signal(value)
        # self.info('routing %s, %s' % (t, name))
        n = 0
        for (translate, b, _) in self.routing:
            if not name in translate:
                continue
            name2 = translate[name]
            x = t, (name2, ob)
            warnings.warn('check this')
            b.put(x, block=False)
            n += 1
        # if n == 0:
            # self.info('no route for %s' % name)
            
        self._pump()

    def end_input(self):
        # self.info('Signaled end of input. Telling a (%s)' % type(self.a))
        for b in self.boxes:
            b.end_input()

        self._pump()

    def _pump(self):
        new_obs = []
        for i, b in enumerate(self.boxes):
            ni = self._pump_one(i, b)
            new_obs.extend(ni)

        # Sort by timestamp
        s = sorted(new_obs, key=lambda x: x[0])
        for x in s:
            self.append(x)

        if all(self.finished):
            self._finished = True

    def _pump_one(self, i, b):
        if self.finished[i]: return
        translate = self.routing[i][2]
        new_obs = []
        while True:
            try:
                x = b.get(block=False)
                t, (name, value) = explode_signal(x)
                name2 = translate[name]
                x2 = t, (name2, value)
                new_obs.append(x2)
            except NotReady:
                break
            except Finished:
                self.finished[i] = True
                break
        return new_obs


def explode_signal(value):
    if not (isinstance(value, tuple) and len(value) == 2 and isinstance(value[1], tuple) and len(value[1]) == 2):
        msg = 'Expected (time, (signal, value)), got %s' % describe_value(value)
        raise ValueError(msg)
    return value



