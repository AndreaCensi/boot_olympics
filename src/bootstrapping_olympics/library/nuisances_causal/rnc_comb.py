from blocks import (Identity, Route, SimpleBlackBoxT, SimpleBlackBoxTN, 
    WrapTMfromT, series)
from bootstrapping_olympics import (RepresentationNuisanceCausal, 
    RepresentationNuisanceCausalIdentity)
from contracts import check_isinstance, contract
from streamels import BootSpec
import warnings


__all__ = [
    'RepNuisanceCausalComb',
    'series_rnc',
]


@contract(returns='isinstance(RepresentationNuisanceCausal)',
          rs='seq(isinstance(RepresentationNuisanceCausal))')
def series_rnc(*rs):
    """ Returns the composition nuisances. """
    if len(rs) == 0:
        return RepresentationNuisanceCausalIdentity()
    if len(rs) == 1:
        one = rs[0]
        check_isinstance(one, RepresentationNuisanceCausal)
        return one
    else:
        first = rs[0]
        check_isinstance(rs[0], RepresentationNuisanceCausal)
        rest = tuple(rs[1:])
        return RepNuisanceCausalComb(first, series_rnc(*rest))


class RepNuisanceCausalComb(RepresentationNuisanceCausal):

    @contract(r1=RepresentationNuisanceCausal,
              r2=RepresentationNuisanceCausal)
    def __init__(self, r1, r2):
        self.r1 = r1
        self.r2 = r2

    def inverse(self):
        r1i = self.r1.get_inverse()
        r2i = self.r2.get_inverse()
        return RepNuisanceCausalComb(r1i,r2i)

    @contract(spec=BootSpec, returns=BootSpec)
    def transform_spec(self, spec):
        spec1 = self.r1.transform_spec(spec)
        spec2 = self.r2.transform_spec(spec1)
        return spec2

    def get_G(self):
        warnings.warn('check the order')
        g1 = self.r1.get_G()
        g2 = self.r2.get_G()
        return series(g1, g2)

    def get_L(self):
        G2 = self.r2.get_G() 
        assert isinstance(G2, SimpleBlackBoxT) and not isinstance(G2, SimpleBlackBoxTN)
        L1 = self.r1.get_L()
        L2 = self.r2.get_L()
        #                 commands
        #                    v     |
        #                   G2     |
        #                    |     |
        #                   cmd1             |
        #                    v               v
        # observations ->  |L1| --> obs1 --> |L2| --> observations
        #

        # This can be written with 3 Route blocks

        #
        #   --> commands       ----------> commands
        #                      ---> G2 --> cmd1
        #       observations   ----------> observations


        r1 = Route([({'commands':'commands'}, Identity(), {'commands':'commands'}),
                    ({'observations':'observations'}, Identity(), {'observations':'observations'}),
                    ({'commands':'commands'}, WrapTMfromT(G2), {'commands': 'cmd1'})])


        #   --> commands        --> cmd1:commands -> |L1| --> obs1
        #                           observations
        #       observations    --> Identity ----------> observations
        #       cmd1            --> Identity ----------> cmd1

        r2 = Route([({'observations':'observations'}, Identity(), {'observations': 'observations'}),
                    ({'cmd1':'commands',
                      'observations':'observations'}, L1, {'observations':'obs1'})])

        #   --> obs1:observations   --> |H2|
        #   -->   commands:commands -->

        r3 = Route([({'obs1':'observations', 'commands':'commands'}, 
                     L2, {'observations':'observations'})])

        return series(r1, r2, r3)

    def get_G_conj(self):
        g1c = self.r1.get_G_conj()
        g2c = self.r2.get_G_conj()
        return series(g2c, g1c)

    def get_L_conj(self):
        raise NotImplementedError(type(self))

