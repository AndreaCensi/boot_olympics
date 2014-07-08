from contracts import contract

from blocks import SimpleBlackBox
from blocks.library import Identity, Route
from bootstrapping_olympics import RepresentationNuisanceCausal
from streamels import BootSpec


__all__ = ['RNCIdentity']


class RNCIdentity(RepresentationNuisanceCausal):
    """ The Identity for RepresentationNuisanceCausal """

    def __init__(self):
        pass
    
    def inverse(self):
        return RNCIdentity()

    @contract(spec=BootSpec, returns=BootSpec)
    def transform_spec(self, spec):
        return spec

    @contract(returns=SimpleBlackBox)
    def get_G(self):
        return Identity()

    @contract(returns=SimpleBlackBox)
    def get_G_conj(self):
        return Identity()

    @contract(returns=SimpleBlackBox)
    def get_H(self):
        # ignore the "commands" signal
        r = Route([({'observations':'observations'},
                    Identity(),
                    {'observations':'observations'})])
        return r

    @contract(returns=SimpleBlackBox)
    def get_H_conj(self):
        # ignore the "commands" signal
        r = Route([({'observations':'observations'},
                    Identity(),
                    {'observations':'observations'})])
        return r
