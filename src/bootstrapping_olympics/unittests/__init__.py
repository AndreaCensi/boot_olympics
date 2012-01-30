"""

    To run these tests with custom config, use:
    
        BO_TEST_CONFIG=config/ nosetests bootstrapping_olympics -v
        
    To add also the default config:
    
        BO_TEST_CONFIG=config/:default nosetests bootstrapping_olympics -v

    You might want to include the vehicles config:
    
        export BO_TEST_CONFIG=config/:default
        export VEHICLES_TEST_CONFIG=config/:default
        nosetests bootstrapping_olympics -v

"""

print("Initializing tests")

_multiprocess_can_split_ = True # Run parallel tests


from .. import logger, np
from .instantiation import *
from .tests_generation import *
from .test_agent import *
from .test_robot import *
from .test_joint import *


from .logs_tests  import *
from .rep_nuisances_tests  import *
from .boot_spec_tests  import *




