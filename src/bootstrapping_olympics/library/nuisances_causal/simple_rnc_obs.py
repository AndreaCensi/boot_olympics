from contracts import contract

from blocks import SimpleBlackBox
from blocks.library import Identity, InstantaneousF, WrapTimedNamed, Route
from bootstrapping_olympics import RepresentationNuisance
from bootstrapping_olympics import RepresentationNuisanceCausal
from streamels import BootSpec


__all__ = ['SimpleRNCObs']


class SimpleRNCObs(RepresentationNuisanceCausal):
    """ A nuisance where there is only a transformation of y defined. """

    @contract(T=RepresentationNuisance)
    def __init__(self, t):
        assert isinstance(t, RepresentationNuisance)
        self.t = t

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


