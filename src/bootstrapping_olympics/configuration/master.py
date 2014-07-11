from contracts import contract

from conf_tools import ConfigMaster, check_generic_code_desc, ObjectSpec


__all__ = [
   'get_boot_config',
   'get_conftools_agents',
   'get_conftools_robots',
   'get_conftools_nuisances',
   'get_conftools_nuisances_causal',
]


@contract(returns=ObjectSpec)
def get_conftools_agents():
    return get_boot_config().agents

@contract(returns=ObjectSpec)
def get_conftools_robots():
    return get_boot_config().robots

@contract(returns=ObjectSpec)
def get_conftools_nuisances():
    return get_boot_config().nuisances

@contract(returns=ObjectSpec)
def get_conftools_nuisances_causal():
    return get_boot_config().nuisances_causal


class BootConfigMaster(ConfigMaster):
    """
    
        *@DynamicAttrs*
    """
    def __init__(self):
        ConfigMaster.__init__(self, 'BootOlympics')

        from bootstrapping_olympics import PassiveRobotInterface
        from bootstrapping_olympics import RepresentationNuisanceCausal
        from bootstrapping_olympics import RepresentationNuisance
        # from bootstrapping_olympics import AgentInterface
        from bootstrapping_olympics import BasicAgent
        from bootstrapping_olympics import LivePlugin
 
        self.robots = self.add_class_generic('robots', '*.robots.yaml', PassiveRobotInterface)

        self.agents = self.add_class_generic('agents', '*.agents.yaml', BasicAgent)

        self.nuisances = self.add_class_generic('nuisances', '*.nuisances.yaml', RepresentationNuisance)
        self.nuisances_causal = self.add_class_generic('nuisances_causal',
                                                '*.nuisances_causal.yaml', RepresentationNuisanceCausal)

        self.videos = self.add_class('videos', '*.videos.yaml', check_valid_videos_config)

        self.live_plugins = self.add_class_generic('live_plugins',
                                                   '*.live_plugins.yaml', LivePlugin)

        
#         v = Constants.TEST_ADDITIONAL_CONFIG_DIR_ENV
#         if v in os.environ:
#             logger.info('Loading configuration according to env var %s:' % v)
# 
#             for dirname in os.environ[v].split(':'):
#                 if dirname == 'default':
#                     logger.info('Loading default config.')
#                     self.load()
#                 else:
#                     logger.info('Using additional dir %r' % dirname)
#                     self.load(dirname)
#         else:
#             # logger.debug('You can use the environment variable %r to preload '
#             #             'the configuration in that directory.' % v)
#             pass

    def get_default_dir(self):
        from pkg_resources import resource_filename  # @UnresolvedImport
        return resource_filename("bootstrapping_olympics", "configs")


get_boot_config = BootConfigMaster.get_singleton



def check_valid_videos_config(spec):
    check_generic_code_desc(spec, 'video')
 



