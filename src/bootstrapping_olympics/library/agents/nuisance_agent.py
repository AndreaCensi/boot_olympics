from .nuisance_agent_actions import (wrap_agent_explorer, wrap_agent_learner, 
    wrap_agent_servo)
from blocks import SimpleBlackBoxTN, Sink
from bootstrapping_olympics import (BasicAgent, ExploringAgent, LearningAgent, 
    PredictingAgent, RepresentationNuisanceCausal, ServoingAgent, 
    get_conftools_agents)
from contracts import contract
from contracts.utils import check_isinstance


__all__ = ['NuisanceAgent']


class NuisanceAgent(BasicAgent, 
                    LearningAgent, ServoingAgent, PredictingAgent,
                    ExploringAgent):
    """ An agent that sees the data filtered through the given nuisances."""
    
    @contract(nuisances='list(str|code_spec|isinstance(RepresentationNuisanceCausal)'
                        '|isinstance(RepresentationNuisance))',
              agent='str|code_spec|isinstance(AgentInterface)')
    def __init__(self, nuisances, agent):
        self.ns = []
        from bootstrapping_olympics.library.agents.nuisance_agent_actions \
            import instance_nuisance_series
        self.nuisance = instance_nuisance_series(nuisances)
        
        check_isinstance(self.nuisance, RepresentationNuisanceCausal)

        config_agents = get_conftools_agents()
        _, self.agent = config_agents.instance_smarter(agent)
        self.spec2 = None

        self.log_add_child('nuisance', self.nuisance)
        self.log_add_child('agent', self.agent)
        
    def init(self, boot_spec):
        self.spec2 = self.nuisance.transform_spec(boot_spec)
        self.agent.init(self.spec2)

    def display(self, report):
        with report.subsection('nuisances') as s:
            for i, n in enumerate(self.ns):
                with s.subsection('n%d' % i) as ss:
                    ss.text('nuisance', str(n))

        self.agent.display(report)

    @contract(returns=SimpleBlackBoxTN)
    def get_explorer(self):
        if self.spec2 is None:
            msg = 'Forgot to call init() before get_explorer().'
            raise ValueError(msg)
        explorer = self.agent.get_explorer()
        return wrap_agent_explorer(explorer, self.nuisance)

    @contract(returns=SimpleBlackBoxTN)
    def get_servo_system(self):
        if self.spec2 is None:
            msg = 'Forgot to call init() before get_servo_system().'
            raise ValueError(msg)
        servo_system = self.agent.get_servo_system()
        return wrap_agent_servo(servo_system, self.nuisance)

    @contract(returns=SimpleBlackBoxTN)
    def get_predictor(self):
        if self.spec2 is None:
            msg = 'Forgot to call init() before get_predictor().'
            raise ValueError(msg)
        # TODO:
        raise NotImplementedError('todo')

    # Learner
  
    @contract(returns=Sink)
    def get_learner_as_sink(self):
        if self.spec2 is None:
            msg = 'Forgot to call init() before get_learner_as_sink().'
            raise ValueError(msg)
        learner = self.agent.get_learner_as_sink()
        
        return wrap_agent_learner(learner, self.nuisance)
  
    def merge(self, other):  # @UnusedVariable
        assert isinstance(other, NuisanceAgent)
        self.agent.merge(other.agent)

    def parallel_process_hint(self, i, n):  # @UnusedVariable
        self.agent.parallel_process_hint(i, n)

    # multilevel
    def need_another_phase(self):
        return self.agent.need_another_phase()

    def start_next_phase(self):
        self.agent.start_next_phase()
        
        