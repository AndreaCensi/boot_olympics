import warnings

from contracts import contract

from blocks.library import Identity, Instantaneous
from bootstrapping_olympics import (AgentInterface, NuisanceNotInvertible,
                                    RepresentationNuisanceCausal, UnsupportedSpec)
import numpy as np
from streamels import BootSpec, StreamSpec, make_streamels_float

from .multilevel_agent import MultiLevelBase
from blocks.library.route import Route


__all__ = ['CmdNormalizeMax']


class CmdNormalizeMax(MultiLevelBase):
    """ 
        Learns the bounds and normalizes the values
        between [-1,+1]. 
    """

    @contract(nsamples='int,>0')
    def __init__(self, nsamples):
        self.nsamples = nsamples

    def init(self, boot_spec):
        if len(boot_spec.get_observations().shape()) != 1:
            raise UnsupportedSpec('I assume 1D signals.')

        self.num = 0
        self.y_max_abs = None

    def merge(self, other):
        if other.num > 0:
            if self.num > 0:
                self.num += other.num
                self.y_max_abs = np.maximum(other.y_max_abs, self.y_max_abs)
            else:
                self.num = other.num
                self.y_max_abs = other.y_max_abs

    def process_observations(self, bd):
        if self.num >= self.nsamples:
            msg = 'I saw %d samples -- converging' % self.num
            self.info(msg)
            raise AgentInterface.LearningConverged(msg)

        y = bd['observations']
        # u = bd['commands']

        y_abs = np.abs(y)
        if self.y_max_abs is None:
            self.y_max_abs = y_abs
        else:
            self.y_max_abs = np.maximum(self.y_max_abs, y_abs)

        self.num += 1

    def get_transform(self):
        return NormalizeMin(self.y_max_abs)

    def display(self, report):
        report.text('nobs', self.num)
        with report.plot('y_max_abs') as pylab:
            pylab.plot(self.y_max_abs, '.')


class NormalizeMin(RepresentationNuisanceCausal):

    def __init__(self, y_max_abs):
        self.y_max_abs = y_max_abs

    def inverse(self):
        raise NuisanceNotInvertible()

    def transform_spec(self, spec):
        shape = self.y_max_abs.shape
        streamels = make_streamels_float(shape, -1, +1, 0)
        boot_spec = BootSpec(obs_spec=StreamSpec('st', streamels),
                             cmd_spec=spec.get_commands())
        return boot_spec

    def get_G(self):
        return Identity()

    def get_G_conj(self):
        return Identity()

    def get_H(self):
        # H must receive 'observations' and 'commands'
        H0 = Rescale(self.y_max_abs)
        return  Route([({'observations':'observations'}, H0,
                     {'observations':'observations'})])

    def get_H_conj(self):
        warnings.warn('check nan')
        Hconj0 = Rescale(1.0 / self.y_max_abs)
        return  Route([({'observations':'observations'}, Hconj0,
                     {'observations':'observations'})])


class Rescale(Instantaneous):
    
    @contract(scale='array[N]')
    def __init__(self, scale):
        Instantaneous.__init__(self)
        self.scale = scale

    @contract(value='tuple(float, tuple(str, array[N]))')
    def transform_value(self, value):
        t, (name, x) = value
        if not x.shape == self.scale.shape:
            msg = 'Invalid shape: %s != %s' % (x.shape, self.scale.shape)
            raise ValueError(msg)
        return(t, (name, x * self.scale))



