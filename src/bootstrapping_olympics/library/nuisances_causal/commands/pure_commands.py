from bootstrapping_olympics import BootWithInternalLog
from collections import namedtuple
from contracts import contract, new_contract


PureCommandsLast = namedtuple('PureCommandsLast',
                              'y0 y1 delta commands commands_index queue_len')

new_contract('PureCommandsLast', PureCommandsLast)


class PureCommands(BootWithInternalLog):
    """ 
        Converts a stream of observations/commands pairs to
        discrete (y0, y1, commands), provided that the commands
        were held for more than delta. 
        
        Note that in case of ``U U U U``, this will return [U U], [U U U], [U U U...]. 
        unless new_behavior=True is given.
        
    """
    def __init__(self, delta, new_behavior=False):
        """ :param delta: minimum length for commands to be the same """
        self.delta = delta
        self.cmdstr2index = {}
        self.reset()
        self.new_behavior = new_behavior
        
    def reset(self):
        self.q = []
        self.cmd = None

    def get_status_string(self):
        s = 'PureCommands: %d in queue' % len(self.q)
        if len(self.q) > 0:
            t0, _ = self.q[0]
            t1, _ = self.q[-1]
            s += ' (%.3f -> %.3f)' % (t0, t1)
        return s

    def update(self, time, commands, y):
        if self.cmd is None:
            # self.info('Starting tracking command %r' % commands)
            self.cmd = commands
        else:
            if cmd2key(self.cmd) != cmd2key(commands):
                # self.info('Switching from %r to %r' % (self.cmd, commands))
                self.q = []
                self.cmd = commands

        if self.q:
            tlast, _ = self.q[-1]
            if tlast == time:
                msg = 'Repeated entry with same time %r.' % tlast
                msg += ' Ignoring.'
                self.warn(msg)
                return
        self.q.append((time, y))

        # print('Time: %s' % [x[0] for x in self.q])
        # t0, _ = self.q[0]
        # t1, _ = self.q[-1]
        # self.info('Now: %d elements, len= %.3f (t0: %.3f t1: %.3f) delta= %s' % 
        # (len(self.q), t1 - t0, t0, t1, self.delta))

    @contract(returns='None|PureCommandsLast')
    def last(self):
        ''' Returns None if not ready; otherwise it returns a tuple 
            of type PureCommandsLast. '''
        if len(self.q) <= 1:
            return None

        t0, y0 = self.q[0]
        t1, y1 = self.q[-1]

        length = t1 - t0
        
        eps = 0.00001
        if length < self.delta - eps:
            # self.info('length = %.3f < %.3f, return None' % (length, self.delta))
            return None

        commands = self.cmd
        commands_index = self.cmd2index(commands)
        
        res = PureCommandsLast(y0=y0, y1=y1, delta=length, commands=commands,
                    commands_index=commands_index,
                    queue_len=(len(self.q) + 1))

        if not self.new_behavior:
            # remove first
            self.q.pop(0)
        else:
            self.q = [self.q[-1]]
            
        return res 
        
    def cmd2index(self, commands):
        key = "%s" % commands
        if not key in self.cmdstr2index:
            self.cmdstr2index[key] = len(self.cmdstr2index)
        return self.cmdstr2index[key]


def cmd2key(x):
    return "%s" % x.tolist()
