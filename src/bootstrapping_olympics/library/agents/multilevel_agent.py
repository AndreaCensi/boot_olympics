from abc import abstractmethod

from contracts import contract, describe_value

from blocks import Sink
from bootstrapping_olympics import (AgentInterface, PassiveAgentInterface,
                            get_conftools_agents, RepresentationNuisanceCausal)
from blocks.library.with_queue import WithQueue


__all__ = ['MultiLevelAgent', 'MultiLevelBase']

class MultiLevelBase(PassiveAgentInterface):

    @abstractmethod
    @contract(returns=RepresentationNuisanceCausal)
    def get_transform(self):
        """ Returns the nuisance at the end of learning. """


class MultiLevelAgent(AgentInterface):
    ''' 
        This is an agent class that allows learning of multi-level
        representations.
        
        The first learner agent represents the first level
        and it is a normal agent.
        
        The first learner must return a Nuisance that transforms 
        the data for the second learner.
        
        It needs to implement the function
        
            get_transform() -> Nuisance
        
        The second learner is any agent using the normal interface. 
        
        This learner has 2 phases.
        In the first phase (phase=0) 
        
        We switch from phase 1 to 2:
        - We decide: If the first agent throws LearningConverged.
          To signal this we throw NextLearningPhase.
        

    '''

    @contract(first='str|code_spec|isinstance(MultiLevelBase)',
               second='str|code_spec|isinstance(PassiveAgentInterface)')
    def __init__(self, first, second):
        config = get_conftools_agents()
        _, self.first = config.instance_smarter(first)
        _, self.second = config.instance_smarter(second)

        if not isinstance(self.first, MultiLevelBase):
            msg =('Expected instance of MultiLevelBase, got %s.' % 
                   describe_value(self.first))
            raise ValueError(msg)

    def init(self, boot_spec):
        self.first_converged = False
        self.boot_spec = boot_spec
        self.first.init(boot_spec)

    def process_observations(self, bd):
        raise ValueError('BUG: Should not have got here; should use learner_system')

    @contract(returns=Sink)
    def get_learner_as_sink(self):
        # something in which we push dict(commands=<>, observations=<>)
        return MultiLevelAgentLearner(self, self.boot_spec)

    def get_predictor(self):
        pass
#         # XXX: needs to concatenate
#         return concat(self.get_nuisance(),
#                       self.second.get_predictor())

    def get_servo(self):
        pass
#         return concat(self.get_nuisance(),
#                       self.second.get_servo())

    def merge(self, other):  # @UnusedVariable
        if not self.first_converged:
            self.first.merge(other.first)
        else:
            self.second.merge(other.second)


class MultiLevelAgentLearner(Sink):
    def __init__(self, ma, boot_spec):
        self.ma = ma
        self.sink1 = ma.first.get_learner_as_sink()
        self.sink2 = None
        self.boot_spec = boot_spec
        
    def put(self, value, block=True, timeout=None):
        if not self.ma.first_converged:
            try:
                self.sink1.put(value, block, timeout)
            except AgentInterface.LearningConverged:
                self.ma.first_converged = True
                
                transform = self.ma.first.get_transform()
                assert isinstance(transform, RepresentationNuisanceCausal)
                self.boot_spec2 = transform.transform_spec(self.boot_spec)
                self.ma.second.init(self.boot_spec2)
                
                G = transform.get_G()
                Gc = transform.get_G_conj()
                H = transform.get_H()
                Hc = transform.get_H_conj()
                self.info('G: %s' % G)
                self.info('H: %s' % H)
                self.info('Gc: %s' % Gc)
                self.info('Hc: %s' % Hc)
                
                
                learner = self.ma.second.get_learner_as_sink()
                Split(learner)
                
                self.sink2 = 
        else:
            # XXX: implement transform
            self.sink2.put(value, block)


class BootExpand(WithQueue):
    def put_noblock(self, value):
        t, bd = value
        self.append((t, ('commands', bd['commands'])))
        self.append((t, ('observations', bd['observations'])))

class Collect(WithQueue):
    """ Collects all the signals with the same timestamp. """
    def __init__(self):
        # XXX
        self.last = {}
        self.last_t = None
    def put_noblock(self, value):
        t, (s, x) = value
        
        if self.last_t is not None:
            if t > self.last_t:
                self.append((t, self.last))
                self.last = {}
            else:
                assert not s in self.last
                self.last[s] = x
        
        self.last_t = t
        

