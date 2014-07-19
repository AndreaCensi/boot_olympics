import warnings

from contracts import contract

from blocks import NotReady, Finished

from blocks.library import WithQueue
from blocks.utils import check_reset
from blocks.exceptions import NeedInput
from blocks.library.timed.checks import check_timed_named
from contracts.interface import describe_value, describe_type
from blocks.interface import SimpleBlackBoxTN


__all__ = ['Route']


class Route(WithQueue, SimpleBlackBoxTN):
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

    def __repr__(self):
        cont = '|'.join([str(b)for b in self.boxes])
        return 'Route(%s)' % cont

    def set_names(self, names):
        assert len(names) == len(self.boxes)
        for i, b in enumerate(self.boxes):
            self.log_add_child(names[i], b)


    def reset(self):
        WithQueue.reset(self)
        for b in self.boxes:
            b.reset()

        self.finished = [False for _ in range(len(self.boxes))]

    def put_noblock(self, value):
        check_reset(self, 'finished')

        check_timed_named(value, self)
        t, (name, ob) = value
        # self.info('routing %s, %s' % (t, name))
        n = 0
        for (translate, b, _) in self.routing:
            if not name in translate:
                continue
            name2 = translate[name]
            x = t, (name2, ob)
            warnings.warn('check this')
            try:
                b.put(x, block=True)
            except NotReady:
                raise
            except BaseException:
                msg = 'Error while trying to put into %r.' % self.log_child_name(b)
                msg += '\n   child: %s' %  describe_value(b)
                msg += '\n  of type %s' %  describe_type(b)
                msg += '\n x = %s' % describe_value(x)
                self.error(msg)
                raise
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
        # self.info('new_obs: %s' % new_obs)
        s = sorted(new_obs, key=lambda x: x[0])
        # self.info('sorted: %s' % s)
        for x in s:
            self.append(x)

        if all(self.finished):
            self._finished = True

    def _pump_one(self, i, b):
        if self.finished[i]: 
            return
        translate = self.routing[i][2]
        new_obs = []
        while True:
            try:
                x = b.get(block=True)
                check_timed_named(x, self)
                t, (name, value) = x
                if not name in translate:
                    msg = 'A block originated an unknown signal.'
                    msg +='\n b = %s' % describe_value(b)
                    msg +='\n signal = %s' % name
                    msg +='\n routing  in = %s' % self.routing[i][0]
                    msg +='\n routing out = %s' % self.routing[i][2]
                    self.info(msg)
                    raise ValueError(msg)
                name2 = translate[name]
                x2 = t, (name2, value)
                new_obs.append(x2)
            except NotReady:
                break
            except Finished:
                self.finished[i] = True
                break
            except NeedInput:
                break
        return new_obs



