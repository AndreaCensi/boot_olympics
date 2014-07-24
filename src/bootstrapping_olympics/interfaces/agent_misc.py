# from blocks.library.simple.with_queue import WithQueue
# import warnings
# from bootstrapping_olympics.interfaces.agent import LearningConverged
# from blocks.interface import Sink
# from contracts import contract
# from blocks.library.timed.checks import check_timed_named
from contracts import contract


__all__ = [
#     'ExplorerAsSystem',
#     'LearnerAsSystem',
    'check_inited'
]
# 
# class ExplorerAsSystem(WithQueue):
#     """ 
#         Sink of bd (two fields 'observations','commands','timestamp')
#         and source of array vector of commands. 
#     """
# 
#     def __str__(self):
#         return 'ExplorerAsSystem(%s)' % self.agent
# 
#     def __init__(self, agent):
#         self.log_add_child('agent', agent)
#         self.agent = agent
# 
#     def reset(self):
#         WithQueue.reset(self)
#         warnings.warn('should do something here')
# 
#     def put_noblock(self, value):  # @UnusedVariable
#         check_timed_named(value)
#         timestamp, (signal, ob) = value
#         
#         if not signal in ['observations']:
#             msg = 'Invalid signal %r to explorer.' % signal
#             raise ValueError(msg)
#         
#         if signal == 'observations':
#             if False: 
# #         # START HERE
#                 try:
#                     self.agent.process_observations(ob)
#                 except LearningConverged:
#                     raise
# 
#         cmd = self.agent.choose_commands()
#         self.append((timestamp, ('commands', cmd)))
# 
# 
# class LearnerAsSystem(Sink):
#     """ Sink of bd (two fields 'observations','commands','timestamp'. """
#     def __init__(self, agent):
#         self.agent = agent
# 
#     def reset(self):
#         warnings.warn('should do something here')
# 
#     def put(self, value, block=True, timeout=None):  # @UnusedVariable
#         timestamp, bd = value  # @UnusedVariable
#         if not 'observations' in bd and 'commands' in bd:
#             msg = 'Expected obs/commands, got: %s' % bd
#             raise ValueError(msg)
#         try:
#             self.agent.process_observations(bd)
#         except LearningConverged:
#             raise


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
