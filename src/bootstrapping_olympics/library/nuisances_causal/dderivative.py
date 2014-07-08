import warnings

from contracts import contract

from blocks import SimpleBlackBox
from blocks.composition import series
from blocks.library import (Identity, WrapTimedNamed, Route, SampledDeriv,
                            WrapTMfromT, Instantaneous)
from bootstrapping_olympics import RepresentationNuisanceCausal
from streamels import (BootSpec, StreamSpec, CompositeStreamSpec,
    UnsupportedSpec, check_streamels_continuous)
from blocks.library.sampled_deriv import SampledDerivPartial


__all__ = ['DDerivative', 'DDerivativeConj']


class DDerivative(RepresentationNuisanceCausal):
    """ A nuisance where there is only a transformation of y defined. """

    def __init__(self):
        pass

    def inverse(self):
        return DDerivativeConj()

    @contract(spec=BootSpec, returns=BootSpec)
    def transform_spec(self, spec):
        cmd = spec.get_commands()
        obs = spec.get_observations()
        
        if not isinstance(obs, StreamSpec):
            msg = 'Need simple streamels as observations'
            raise UnsupportedSpec(msg)
        check_streamels_continuous(obs.get_streamels())
        
        streamels = obs.get_streamels()
        signal = StreamSpec(id_stream='signa', streamels=streamels)
        signal_deriv = StreamSpec(id_stream='signal_deriv', streamels=streamels)
        obs2 = CompositeStreamSpec(dict(signal=signal, signal_deriv=signal_deriv))

        return BootSpec(obs2, cmd)

    @contract(returns=SimpleBlackBox)
    def get_G(self):
        return Identity()

    @contract(returns=SimpleBlackBox)
    def get_G_conj(self):
        return Identity()

    @contract(returns=SimpleBlackBox)
    def get_H(self):
        w = WrapTMfromT(series(SampledDeriv(), (MakeDict())))

        # ignore the "commands" signal
        r = Route([({'observations':'observations'},
                    w,
                    {'observations':'observations'})])
        return r

    @contract(returns=SimpleBlackBox)
    def get_H_conj(self):
        # XXX:
        warnings.warn(' this is not correct')
        w = WrapTMfromT(WrapTimedNamed(ExtractField('signal')))
        # ignore the "commands" signal
        r = Route([({'observations':'observations'},
                    w,
                    {'observations':'observations'})])
        return r


class MakeDict(Instantaneous):
    @contract(value='tuple(float, tuple(array, array))')
    def transform_value(self, value):
        t, x = value
        assert isinstance(x, tuple) and len(x) == 2
        res = dict(signal=x[0], signal_deriv=x[1])
        return t, res

class ExtractField(Instantaneous):
    def __init__(self, field):
        self.field = field
    def transform_value(self, x):
        return x[self.field]


class DDerivativeConj(RepresentationNuisanceCausal):
    pass


