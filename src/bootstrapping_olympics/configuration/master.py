from . import logger
from .. import Constants
from ..interfaces import AgentInterface, RobotInterface, RepresentationNuisance
from .agents import check_valid_agent_config
from .events import check_valid_event_config
from .robots import check_valid_robot_config, check_valid_nuisance_config
from .tasks import check_valid_task_config
from conf_tools import ConfigMaster, GenericInstance
import os

class BootConfigMaster(ConfigMaster):
    def __init__(self):
        ConfigMaster.__init__(self, logger)

        self.add_class('robots', '*.robots.yaml', check_valid_robot_config,
                       GenericInstance(RobotInterface))
        self.add_class('agents', '*.agents.yaml', check_valid_agent_config,
                       GenericInstance(AgentInterface))
        self.add_class('nuisances', '*.nuisances.yaml',
                       check_valid_nuisance_config,
                       GenericInstance(RepresentationNuisance))

        self.add_class('events', '*.events.yaml', check_valid_event_config)
        self.add_class('tasks', '*.tasks.yaml', check_valid_task_config)
        
        self.robots = self.specs['robots']
        self.agents = self.specs['agents']
        self.events = self.specs['events']
        self.tasks = self.specs['tasks']
        self.nuisances = self.specs['nuisances']
        
        v = Constants.TEST_ADDITIONAL_CONFIG_DIR_ENV
        if v in os.environ:
            logger.info('Loading configuration according to env var %s:' % v)

            for dirname in os.environ[v].split(':'):
                if dirname == 'default':
                    logger.info('Loading default config.')
                    self.load()
                else:
                    logger.info('Using additional dir %r' % dirname)
                    self.load(dirname)             
        else:
            logger.info('Use env var %s to add predefined config dirs.' % v)


    def get_default_dir(self):
        from pkg_resources import resource_filename #@UnresolvedImport
        return resource_filename("bootstrapping_olympics", "configs")

    
BootOlympicsConfig = BootConfigMaster()
 
