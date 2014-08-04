from bootstrapping_olympics import (RepresentationNuisance, 
    RepresentationNuisanceCausalSimpleInst, get_conftools_nuisances)
from contracts import contract
from contracts.utils import check_isinstance
from streamels import BootSpec


__all__ = [
    'SimpleRNCObs',
]


class SimpleRNCObs(RepresentationNuisanceCausalSimpleInst):
    """ A nuisance where there is only a transformation of y defined. """

    @contract(nuisance='str|code_spec|isinstance(RepresentationNuisance)')
    def __init__(self, nuisance):
        _, self.t = get_conftools_nuisances().instance_smarter(nuisance)

        check_isinstance(self.t, RepresentationNuisance)
        self.log_add_child('t', self.t)
        self.spec = None
        self.t_inv = None

    def inverse(self):
        if self.t_inv is None:
            msg = 'transform_spec() called before inverse().'
            raise ValueError(msg)
        return SimpleRNCObs(self.t.inverse())

    @contract(spec=BootSpec, returns=BootSpec)
    def transform_spec(self, spec):
        self.spec = spec
        cmd = spec.get_commands()
        self.obs = spec.get_observations()
        self.obs2 = self.t.transform_spec(self.obs)
        return BootSpec(self.obs2, cmd)

    def _get_t_inv(self):
        """ Creates the inverse only when it is necessary. """
        if self.t_inv is None:
            self.t_inv = self.t.left_inverse()
            self.log_add_child('t_inv', self.t_inv)
            obsb = self.t_inv.transform_spec(self.obs2)
            assert self.obs == obsb
        return self.t_inv



    def get_h(self):
        self._check_inited()
        return self.t.transform_value

    def get_h_conj(self):
        self._check_inited()
        t_inv = self._get_t_inv()
        return t_inv.transform_value

    def get_g(self):
        self._check_inited()
        return lambda x: x
    
    def get_g_conj(self):
        self._check_inited()
        return lambda x: x

    def _check_inited(self):
        if self.spec is None:
            msg = 'transform_spec() not called yet.'
            raise ValueError(msg)
