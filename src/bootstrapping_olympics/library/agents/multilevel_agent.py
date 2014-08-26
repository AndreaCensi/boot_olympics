from .nuisance_agent_actions import wrap_agent_learner
from abc import abstractmethod
from blocks import SimpleBlackBox, Sink
from bootstrapping_olympics import (BasicAgent, ExploringAgent, LearningAgent, 
    RepresentationNuisanceCausal, ServoingAgent, get_conftools_agents)
from contracts import check_isinstance, contract, describe_type, describe_value


__all__ = [
       'TwoLevelAgent',
       'MultiLevelBase',
       'MultiLevelAgent',
]

class MultiLevelBase(BasicAgent):

    @abstractmethod
    @contract(returns=RepresentationNuisanceCausal)
    def get_transform(self):
        """ Returns the nuisance at the end of learning. """


def MultiLevelAgent(agents):
    if len(agents) == 2:
        return TwoLevelAgent(agents[0], agents[1])
    elif len(agents) > 2:
        first_levels = MultiLevelAgent(agents[:-1])
        last = agents[-1]
        return TwoLevelAgent(first_levels, last)


class TwoLevelAgent(MultiLevelBase, LearningAgent, ExploringAgent, ServoingAgent):
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

        self.log_add_child('first', self.first)
        self.log_add_child('second', self.second)

    def init(self, boot_spec):
        self.phase = 0
        self.boot_spec = boot_spec
        self.first.init(boot_spec)

    def publish(self, r):
        r.text('phase', self.phase)
        with r.subsection('first') as sub:
            self.first.publish(sub)
        with r.subsection('second') as sub:
            self.second.publish(sub)

    def process_observations(self, bd):
        raise ValueError('BUG: Should not have got here; should use learner_system')

    def need_another_phase(self):
        self.info('asked for next phase: current %s' % self.phase)
        if self.phase == 0:
            return True
        else:
            self.info('asked for next phase--answering no')
            return False

    def start_next_phase(self):
        if self.phase == 0:
            if self.first.need_another_phase():
                self.first.start_next_phase()
            else:
                self.phase = 1
        else:
            raise ValueError('should not be here')
        
    @contract(returns=Sink)
    def get_learner_as_sink(self):
        if not isinstance(self.first, LearningAgent):
            msg = ('First agent is not a LearningAgent (%s).' % 
                   describe_type(self.first))
            raise NotImplementedError(msg)

        # something in which we push dict(commands=<>, observations=<>)
        self.info('get_learner_as_sink() for phase=%d' % self.phase)

        if self.phase == 0:
            return self.first.get_learner_as_sink()
        elif self.phase == 1:
            if not isinstance(self.second, LearningAgent):
                msg = ('second agent is not a LearningAgent (%s).' % 
                       describe_type(self.second))
                raise NotImplementedError(msg)
            
            transform = self.first.get_transform()
            assert isinstance(transform, RepresentationNuisanceCausal)
            boot_spec2 = transform.transform_spec(self.boot_spec)
            self.second.init(boot_spec2)
            learner = self.second.get_learner_as_sink()
            self.info('wrapping learner %s' % learner)
            return wrap_agent_learner(learner, transform)

    @contract(returns=SimpleBlackBox)
    def get_explorer(self):
        # TODO: different explorer for different phases?
        if not isinstance(self.first, ExploringAgent):
            msg = ('First agent is not an ExploringAgent (%s).' % 
                   describe_type(self.first))
            raise NotImplementedError(msg)
        return self.first.get_explorer()

    def get_predictor(self):
        # TODO: implement
        raise NotImplementedError(type(self))

    def get_servo(self):
        # TODO: implement
        raise NotImplementedError(type(self))
    
    @contract(returns=SimpleBlackBox)
    def get_servo_system(self):
        # TODO: check that we have converged
        raise NotImplementedError(type(self))

    def merge(self, other):  # @UnusedVariable
        if self.phase == 0:
            self.info('merge() for phase=%d' % self.phase)
            self.first.merge(other.first)
        else:
            self.info('merge() for phase=%d' % self.phase)
            self.second.merge(other.second)

    @contract(returns=RepresentationNuisanceCausal)
    def get_transform(self):
        check_isinstance(self.second, MultiLevelBase)
        t1 = self.first.get_transform()
        t2 = self.second.get_transform()
        from bootstrapping_olympics.library.nuisances_causal import series_rnc
        return series_rnc(t1, t2)
