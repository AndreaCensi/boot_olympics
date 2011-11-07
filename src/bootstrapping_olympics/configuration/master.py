from ..interfaces import AgentInterface, RobotInterface, RepresentationNuisance
from .agents import check_valid_agent_config
from .events import check_valid_event_config
from .robots import check_valid_robot_config, check_valid_nuisance_config
from .tasks import check_valid_task_config
from conf_tools import ConfigMaster, GenericInstance

class BootConfigMaster(ConfigMaster):
    def __init__(self):
        ConfigMaster.__init__(self)

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
        
    def get_default_dir(self):
        from pkg_resources import resource_filename #@UnresolvedImport
        return resource_filename("bootstrapping_olympics", "configs")

    
BootOlympicsConfig = BootConfigMaster()

## TODO: deprecated
#Configuration = BootOlympicsConfig
#
## TODO: deprecated
#def load_boot_olympics_config(directory):
#    BootOlympicsConfig.load(directory)
#
## TODO: deprecated             
#load_configuration = load_boot_olympics_config
    
