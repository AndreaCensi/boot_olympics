from contracts import contract

from bootstrapping_olympics import AgentInterface
from bootstrapping_olympics import UnsupportedSpec

from .multilevel_agent import MultiLevelBase


__all__ = ['CmdNormalizeMax']

class CmdNormalizeMax(MultiLevelBase):
    """ 
        Learns the bounds and normalizes the values
        between [-1,+1]. 
    """

    @contract(nsamples='int,>0')
    def __init__(self, nsamples):
        self.nsamples = nsamples

    def init(self, boot_spec):
        if len(boot_spec.get_observations().shape()) != 1:
            raise UnsupportedSpec('I assume 1D signals.')

        self.num = 0

    def process_observations(self, bd):
        if self.num >= self.nsamples:
            msg = 'I saw %d samples -- converging' % self.num
            self.info(msg)
            raise AgentInterface.LearningConverged(msg)

        y = bd['observations']
        u = bd['commands']
    
        self.num += 1

    def get_transform(self):
        pass
