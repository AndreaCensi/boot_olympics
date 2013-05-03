from . import logger
from .. import Constants, LivePlugin
from ..interfaces import AgentInterface, RepresentationNuisance
from conf_tools import ConfigMaster, check_generic_code_desc
from contracts import contract
import os
from bootstrapping_olympics.interfaces.robot import PassiveRobotInterface


class BootConfigMaster(ConfigMaster):
    """
    
        *@DynamicAttrs*
    """
    def __init__(self):
        ConfigMaster.__init__(self, 'BootOlympics')

 
        self.robots = self.add_class_generic('robots', '*.robots.yaml', PassiveRobotInterface)
        self.agents = self.add_class_generic('agents', '*.agents.yaml', AgentInterface)

        self.nuisances = self.add_class_generic('nuisances', '*.nuisances.yaml', RepresentationNuisance)

        self.videos = self.add_class('videos', '*.videos.yaml', check_valid_videos_config)

        self.live_plugins = self.add_class_generic('live_plugins',
                                                   '*.live_plugins.yaml', LivePlugin)

        
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
            # logger.debug('You can use the environment variable %r to preload '
            #             'the configuration in that directory.' % v)
            pass

    def get_default_dir(self):
        from pkg_resources import resource_filename  # @UnresolvedImport
        return resource_filename("bootstrapping_olympics", "configs")


    singleton = None



def check_valid_videos_config(spec):
    check_generic_code_desc(spec, 'video')
 


@contract(returns=BootConfigMaster)
def get_boot_config():
    if BootConfigMaster.singleton is None:
        BootConfigMaster.singleton = BootConfigMaster()
    return BootConfigMaster.singleton 


@contract(c=BootConfigMaster)
def set_boot_config(c):
    BootConfigMaster.singleton = c  


BootOlympicsConfig = get_boot_config()




