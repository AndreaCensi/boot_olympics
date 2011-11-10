from . import DirectoryStructure
from .... import BootOlympicsConfig
from ....agent_states import LearningStateDB
from ....logs import LogIndex

class DataCentral:
    def __init__(self, boot_root=None):
        self.root = boot_root
        self.dir_structure = DirectoryStructure(self.root)
        self.states_db = None
        self.log_index = None
        self.bo_config = None
    
    def get_bo_config(self):
        if self.bo_config is None:
            dirs = self.dir_structure.get_config_directories()
            for dirname in dirs:
                BootOlympicsConfig.load(dirname)
            self.bo_config = BootOlympicsConfig
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

        
        
