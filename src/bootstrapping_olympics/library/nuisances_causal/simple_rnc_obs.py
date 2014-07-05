from contracts import contract
from contracts.utils import check_isinstance

from blocks import SimpleBlackBox
from blocks.library import Identity, InstantaneousF, WrapTimedNamed, Route
from bootstrapping_olympics import RepresentationNuisance, RepresentationNuisanceCausal, get_conftools_nuisances
from streamels import BootSpec


__all__ = ['SimpleRNCObs']


class SimpleRNCObs(RepresentationNuisanceCausal):
    """ A nuisance where there is only a transformation of y defined. """

    @contract(nuisance='str|code_spec|isinstance(RepresentationNuisance)')
    def __init__(self, nuisance):

        _, self.t = get_conftools_nuisances().instance_smarter(nuisance)

        check_isinstance(self.t, RepresentationNuisance)

    def inverse(self):
        return SimpleRNCObs(self.t.inverse())

    @contract(spec=BootSpec, returns=BootSpec)
    def transform_spec(self, spec):
        cmd = spec.get_commands()
        obs = spec.get_observations()
        obs2 = self.t.transform_spec(obs)
        return BootSpec(obs2, cmd)

    @contract(returns=SimpleBlackBox)
    def get_G(self):
        return Identity()

    @contract(returns=SimpleBlackBox)
    def get_G_conj(self):
        return Identity()

    @contract(returns=SimpleBlackBox)
    def get_H(self):
        w = wrap_nuisance(self.t)
        r = Route([({'observations':'observations'}, w, {'observations':'observations'})])
        return r

    @contract(returns=SimpleBlackBox)
    def get_H_conj(self):
        t_inv = self.t.left_inverse()
        w = wrap_nuisance(t_inv)
        r = Route([({'observations':'observations'}, w, {'observations':'observations'})])
        return r


def wrap_nuisance(t):
    f = t.transform_value
    return WrapTimedNamed(InstantaneousF(f))


