from blocks import SimpleBlackBox
from blocks.library import Route
from blocks.library import IdentityTimed
from blocks.library import IdentityTimedNamed
from bootstrapping_olympics import RepresentationNuisanceCausal
from contracts import contract
from streamels import BootSpec
from blocks.interface import SimpleBlackBoxT



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

    @contract(returns=SimpleBlackBoxT)
    def get_G(self):
        return IdentityTimed()

    @contract(returns=SimpleBlackBoxT)
    def get_G_conj(self):
        return IdentityTimed()

    @contract(returns=SimpleBlackBox)
    def get_L(self):
        # ignore the "commands" signal
        r = Route([({'observations':'observations'},
                    IdentityTimedNamed(),
                    {'observations':'observations'})], suppress=['commands'])
        return r

    @contract(returns=SimpleBlackBox)
    def get_L_conj(self):
        # ignore the "commands" signal
        r = Route([({'observations':'observations'},
                    IdentityTimedNamed(),
                    {'observations':'observations'})], suppress=['commands'])
        return r
