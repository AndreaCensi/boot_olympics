from . import (check_valid_agent_config, check_valid_robot_config,
    check_valid_nuisance_config, logger)
from .. import Constants
from ..interfaces import AgentInterface, RobotInterface, RepresentationNuisance
from conf_tools import ConfigMaster, GenericInstance, check_generic_code_desc
import os
from bootstrapping_olympics import LivePlugin
from bootstrapping_olympics.configuration.agents import check_valid_plugin_config


def check_valid_videos_config(spec):
    check_generic_code_desc(spec, 'video')


class BootConfigMaster(ConfigMaster):
    def __init__(self):
        ConfigMaster.__init__(self, 'BootOlympics')

        self.robots = self.add_class('robots', '*.robots.yaml',
                                     check_valid_robot_config,
                       GenericInstance(RobotInterface))

        self.agents = self.add_class('agents', '*.agents.yaml',
                                     check_valid_agent_config,
                       GenericInstance(AgentInterface))

        self.nuisances = self.add_class('nuisances', '*.nuisances.yaml',
                       check_valid_nuisance_config,
                       GenericInstance(RepresentationNuisance))

        self.videos = self.add_class('videos', '*.videos.yaml',
                       check_valid_videos_config)

        self.live_plugins = self.add_class('live_plugins',
                                           '*.live_plugins.yaml',
                                           check_valid_plugin_config,
                                           GenericInstance(LivePlugin))

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
            #logger.debug('You can use the environment variable %r to preload '
            #             'the configuration in that directory.' % v)
            pass

    def get_default_dir(self):
        from pkg_resources import resource_filename #@UnresolvedImport
        return resource_filename("bootstrapping_olympics", "configs")


BootOlympicsConfig = BootConfigMaster()

