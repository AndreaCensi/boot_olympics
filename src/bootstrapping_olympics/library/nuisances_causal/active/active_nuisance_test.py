import unittest

from bootstrapping_olympics import (
     BootSpec, StreamSpec)
from bootstrapping_olympics.library.robots import EquivRobotCausal, RandomRobot
from bootstrapping_olympics.utils import assert_allclose
import numpy as np
from streamels import make_streamels_1D_float

from .active_nuisance import ActiveNuisance


class ActiveNuisanceTest(unittest.TestCase):
    
    def test_pure_commands(self):
        shape = (3,)
        y0 = np.random.rand(*shape)
        
        obs_spec = StreamSpec(id_stream=None,
                              streamels=make_streamels_1D_float(shape[0], 0.0, 1.0))
        cmd_spec = StreamSpec(id_stream=None,
                              streamels=make_streamels_1D_float(2, 0.0, 1.0))
            
        nuisance = ActiveNuisance()
        r0 = RandomRobot(BootSpec(obs_spec, cmd_spec), t0=0.0, y0=y0)
        robot = EquivRobotCausal(r0, nuisance)

        
        # bit off    
        rest1 = np.array([0.5, 0.5, 0.0])
        # bit on
        rest2 = np.array([0.5, 0.5, 1.0])
    
        robot.set_commands(rest1, 'rest')
        obs1 = robot.get_observations()
        
        robot.set_commands(rest2, 'rest')
        obs2 = robot.get_observations()
        
        assert_allclose(obs2.observations, 1 - obs1.observations)    
