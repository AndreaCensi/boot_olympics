from blocks import SimpleBlackBox
from bootstrapping_olympics import (RepresentationNuisance, 
    RepresentationNuisanceCausalSimpleInst, get_conftools_nuisances)
from contracts import contract
from contracts.utils import check_isinstance
from streamels import BootSpec



__all__ = [
    'SimpleRNCCmd',
]


class SimpleRNCCmd(RepresentationNuisanceCausalSimpleInst):
    """ A nuisance where there is only a transformation of y=u defined. """

    @contract(nuisance='str|code_spec|isinstance(RepresentationNuisance)')
    def __init__(self, nuisance):

        _, self.t = get_conftools_nuisances().instance_smarter(nuisance)

        check_isinstance(self.t, RepresentationNuisance)

    def inverse(self):
        return SimpleRNCCmd(self.t.inverse())

    @contract(spec=BootSpec, returns=BootSpec)
    def transform_spec(self, spec):
        cmd = spec.get_commands()
        obs = spec.get_observations()
        cmd2 = self.t.transform_spec(cmd)
        return BootSpec(obs, cmd2)

    def get_g(self):
        return self.t.transform_value

    def get_g_conj(self):
        return self.t.left_inverse().transform_value

    def get_h(self):
        return lambda x: x
    
    def get_h_conj(self):
        return lambda x: x


