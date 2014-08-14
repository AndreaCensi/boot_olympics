from .directory_structure import DirectoryStructure
from bootstrapping_olympics import (LearningStateDB, LogIndex, get_boot_config, 
    logger)
from bootstrapping_olympics.utils import expand_environment
from conf_tools.master import GlobalConfig
from conf_tools.utils import friendly_path
import os

__all__ = ['DataCentral']


class DataCentral(object):
    def __init__(self, boot_root=None):
        boot_root = expand_environment(boot_root)
        # Important, it can be deserialized from somewhere else
        self.boot_root = boot_root
        self.root = os.path.realpath(boot_root)
        if not os.path.exists(self.root):
            try:
                os.makedirs(self.root)
            except:
                if not os.path.exists(self.root):
                    raise
        self.dir_structure = DirectoryStructure(self.root)
        self.states_db = None
        self.log_index = None
        self.bo_config = None

    def get_boot_root(self):
        return self.root
        
    def __repr__(self):
        return 'DataCentral(root=%r)' % self.root

    def get_bo_config(self):
        if self.bo_config is None:
            self.bo_config = get_boot_config()
            
            dirs = self.dir_structure.get_config_directories()
            for dirname in dirs:
                if not os.path.exists(dirname):
                    msg = ('Warning, the config dir %r does not exist ' % 
                           friendly_path(dirname))
                    logger.info(msg)  
                else:
                    GlobalConfig.global_load_dir(dirname)
        return self.bo_config

    def get_log_index(self, ignore_cache=False):
        if self.log_index is None:
            self.log_index = LogIndex()
            log_directories = self.dir_structure.get_log_directories()
            for dirname in log_directories:
                self.log_index.index(dirname, ignore_cache=ignore_cache)
        return self.log_index

    def get_agent_state_db(self):
        if self.states_db is None:
            state_db_dir = self.dir_structure.get_state_db_directory()
            self.states_db = LearningStateDB(state_db_dir)
        return self.states_db

    def get_dir_structure(self):
        return self.dir_structure



