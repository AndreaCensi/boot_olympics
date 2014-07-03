from contracts import contract

from blocks.composition import series
from blocks.library import Identity
from blocks.library import Route
from bootstrapping_olympics import RepresentationNuisanceCausal
from streamels import BootSpec


__all__ = [
    'RepNuisanceCausalComb',
    'series_rnc',
]

@contract(r1=RepresentationNuisanceCausal,
          r2=RepresentationNuisanceCausal)
def series_rnc(r1, r2):
    """ Returns the composition of two nuisances. """
    return RepNuisanceCausalComb(r1, r2)


class RepNuisanceCausalComb(RepresentationNuisanceCausal):
    @contract(r1=RepresentationNuisanceCausal,
              r2=RepresentationNuisanceCausal)
    def __init__(self, r1, r2):
        self.r1 = r1
        self.r2 = r2

    def inverse(self):
        # TODO
        raise NotImplementedError(type(self))

    @contract(spec=BootSpec, returns=BootSpec)
    def transform_spec(self, spec):
        spec1 = self.r1.transform_spec(spec)
        spec2 = self.r2.transform_spec(spec1)
        return spec2

    def get_G(self):
        raise NotImplementedError(type(self))

    def get_H(self):
        G2 = self.r2.get_G()
        H1 = self.r1.get_H()
        H2 = self.r2.get_H()
        #                 commands
        #                    v     |
        #                   G2     |
        #                    |     |
        #                   cmd1             |
        #                    v               v
        # observations ->  |H| --> obs1 --> |H2| --> observations
        #

        # This can be written with 3 Route blocks

        #
        #   --> commands       ----------> commands
        #                      ---> G2 --> cmd1
        #       observations   ----------> observations


        r1 = Route([({'commands':'commands'}, Identity(), {'commands':'commands'}),
                    ({'observations':'observations'}, Identity(), {'observations':'observations'}),
                    ({'commands':'commands'}, G2, {'commands': 'cmd1'})])


        #   --> commands        --> cmd1:commands -> |H1| --> obs1
        #                           observations
        #       observations    --> Identity ----------> observations
        #       cmd1            --> Identity ----------> cmd1

        r2 = Route([({'observations':'observations'}, Identity(), {'observations': 'observations'}),
                    ({'cmd1':'cmd1'}, Identity(), {'cmd1': 'cmd1'}),
                    ({'cmd1':'commands',
                      'observations':'observations'}, H1, {'observations':'obs1'})])

        #   --> obs1:observations   --> |H2|
        #   -->   commands:commands -->

        r3 = Route([({'obs1':'observations', 'commands':'commands'}, H2, {'observations':'observations'})])

        return series(r1, r2, r3)

    def get_G_conj(self):
        g1c = self.r1.get_G_conj()
        g2c = self.r2.get_G_conj()
        return series(g2c, g1c)

    def get_H_conj(self):
        raise NotImplementedError(type(self))

