from blocks.library.with_queue import WithQueue
import warnings
from bootstrapping_olympics.interfaces.agent import LearningConverged
from blocks.interface import Sink
from contracts import contract


__all__ = [
    'ExplorerAsSystem',
    'LearnerAsSystem',
    'check_inited'
]

class ExplorerAsSystem(WithQueue):
    """ 
        Sink of bd (two fields 'observations','commands','timestamp')
        and source of array vector of commands. 
    """

    def __str__(self):
        return 'ExplorerAsSystem(%s)' % self.agent

    def __init__(self, agent):
        self.log_add_child('agent', agent)
        self.agent = agent

    def reset(self):
        WithQueue.reset(self)
        warnings.warn('should do something here')

#     def get(self, block=True, timeout=None):  # @UnusedVariable
#         self.info('calling choose_commands() block = %s' % block)
#         res = self.agent.choose_commands()
#         self.info('result is %s' % str(res))
#         return res

    def put_noblock(self, value):  # @UnusedVariable
        timestamp, bd = value  # @UnusedVariable
        if not 'observations' in bd and 'commands' in bd:
            msg = 'Expected obs/commands, got: %s' % bd
            raise ValueError(msg)

        # self.info('explorer received bd at %s' % timestamp)
        try:
            self.agent.process_observations(bd)
        except LearningConverged:
            raise

        res = self.agent.choose_commands()
        self.append(res)


class LearnerAsSystem(Sink):
    """ Sink of bd (two fields 'observations','commands','timestamp'. """
    def __init__(self, agent):
        self.agent = agent

    def reset(self):
        warnings.warn('should do something here')

    def put(self, value, block=True, timeout=None):  # @UnusedVariable
        timestamp, bd = value  # @UnusedVariable
        if not 'observations' in bd and 'commands' in bd:
            msg = 'Expected obs/commands, got: %s' % bd
            raise ValueError(msg)
        try:
            self.agent.process_observations(bd)
        except LearningConverged:
            raise


@contract(attrname='str')
def check_inited(agent, attrname):
    """ 
        checks that init() has been called on the agent 
        by making sure that there is the given attribute. 
    """
    if not hasattr(agent, attrname):
        msg = 'You forgot to call init() on this agent.\n'
        msg += '\nattribute not present: %r' % attrname
        msg += '\nblock: %s' % agent
        raise Exception(msg)
