from contracts import contract
from contracts.utils import check_isinstance

from blocks import Sink
from bootstrapping_olympics import (get_conftools_nuisances_causal,
    get_conftools_agents, AgentInterface)
from bootstrapping_olympics import RepresentationNuisance, RepresentationNuisanceCausal, get_conftools_nuisances

from .nuisance_agent_actions import wrap_agent_learner


__all__ = ['NuisanceAgent']


class NuisanceAgent(AgentInterface):
    """ An agent that sees the data filtered through the given nuisances."""
    
    @contract(nuisances='list(str|code_spec|isinstance(RepresentationNuisanceCausal)|isinstance(RepresentationNuisance))',
              agent='str|code_spec|isinstance(AgentInterface)')
    def __init__(self, nuisances, agent):
        self.ns = []
        config_nuisances_causal = get_conftools_nuisances_causal()
        config_nuisances = get_conftools_nuisances()
        for n in nuisances:

            if isinstance(n, str):
                c1 = n in config_nuisances
                c2 = n in config_nuisances_causal
                num = (1 if c1 else 0) + (1 if c2 else 0)
                if num == 0:
                    msg = 'Could not find %r as either type of nuisance.' % n
                    raise ValueError(msg)
                if num == 2:
                    msg = 'Ambiguous name %r.' % n
                    raise ValueError(msg)
                if c1:
                    assert not c2
                    _, x = config_nuisances.instance_smarter(n)
                else:
                    _, x = config_nuisances_causal.instance_smarter(n)
                    assert c2
            else:
                try:
                    _, x = config_nuisances_causal.instance_smarter(n)
                except:
                    _, x = config_nuisances.instance_smarter(n)

            if isinstance(x, RepresentationNuisance):
                from bootstrapping_olympics.library.nuisances_causal import SimpleRNCObs
                x = SimpleRNCObs(x)

            check_isinstance(x, RepresentationNuisanceCausal)
            self.ns.append(x)

        from bootstrapping_olympics.library.nuisances_causal import series_rnc
        self.nuisance = series_rnc(*tuple(self.ns))

        check_isinstance(self.nuisance, RepresentationNuisanceCausal)

        config_agents = get_conftools_agents()
        _, self.agent = config_agents.instance_smarter(agent)

    def init(self, boot_spec):
        self.spec2 = self.nuisance.transform_spec(boot_spec)
        self.agent.init(self.spec2)

    def display(self, report):
        with report.subsection('nuisances') as s:
            for i, n in enumerate(self.ns):
                with s.subsection('n%d' % i) as ss:
                    ss.text('nuisance', str(n))

        self.agent.display(report)

    @contract(returns=Sink)
    def get_learner_as_sink(self):
        learner = self.agent.get_learner_as_sink()
        return wrap_agent_learner(learner, self.nuisance)

    def merge(self, other):  # @UnusedVariable
        assert isinstance(other, NuisanceAgent)
        self.agent.merge(other.agent)

    def parallel_process_hint(self, i, n):  # @UnusedVariable
        self.agent.parallel_process_hint(i, n)

    def need_another_phase(self):
        return self.agent.need_another_phase()

    def start_next_phase(self):
        self.agent.start_next_phase()

    def choose_commands(self):
        raise Exception('Using old API with choose_commands().')

    def process_observations(self, obs):
        raise Exception('Using old API with process_observations().')


