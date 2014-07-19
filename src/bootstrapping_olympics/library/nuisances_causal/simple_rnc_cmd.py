from bootstrapping_olympics import (RepresentationNuisance, 
    RepresentationNuisanceCausalSimpleInst, get_conftools_nuisances)
from contracts import contract
from contracts.utils import check_isinstance
from streamels import BootSpec


__all__ = [
    'SimpleRNCCmd',
]


class SimpleRNCCmd(RepresentationNuisanceCausalSimpleInst):
    """ A nuisance where there is only a transformation of commands defined. """

    @contract(nuisance='str|code_spec|isinstance(RepresentationNuisance)')
    def __init__(self, nuisance):
        _, self.t = get_conftools_nuisances().instance_smarter(nuisance)

        check_isinstance(self.t, RepresentationNuisance)
        self.spec = None
        
    def inverse(self):
        return SimpleRNCCmd(self.t.inverse())

    @contract(spec=BootSpec, returns=BootSpec)
    def transform_spec(self, spec):
        self.spec = spec
        cmd = spec.get_commands()
        obs = spec.get_observations()
        cmd2 = self.t.transform_spec(cmd)
        return BootSpec(obs, cmd2)

    def get_g(self):
        self._check_inited()
        return self.t.transform_value

    def get_g_conj(self):
        self._check_inited()
        return self.t.left_inverse().transform_value

    def get_h(self):
        self._check_inited()
        return lambda x: x
    
    def get_h_conj(self):
        self._check_inited()
        return lambda x: x

    def _check_inited(self):
        if self.spec is None:
            msg = 'transform_spec() not called yet.'
            raise ValueError(msg)
