from . import BootOlympicsConfig
from ..utils import expand_environment, substitute
import os

class DirectoryStructure:
    
    DEFAULT_ROOT = '~/boot_olympics/'
    
    def __init__(self, root=None):
        if root is None: 
            root = DirectoryStructure.DEFAULT_ROOT
        self.root = expand_environment(root)
        
        if not os.path.exists(self.root):
            msg = 'The root directory %s does not exist.' % self.root
            raise Exception(msg)
        
    def additional_config_dir(self, dirname):
        assert False
    
    def additional_log_dir(self, dirname):
        assert False
     
    def get_config_directories(self):
        dirs = []
        dirs.append(BootOlympicsConfig.get_default_dir())
        dirs.append(os.path.join(self.root, 'config/'))
        return dirs
         
    def get_log_directories(self):
        ''' Returns a list of the directories where to look for logs. '''
        dirs = []
        dirs.append(os.path.join(self.root, 'logs/'))
        # TODO: additional
        return dirs
    
    def get_state_db_directory(self):
        return os.path.join(self.root, 'states/')

    def get_simlog_hdf_filename(self, id_agent, id_robot, id_stream):
        pattern = 'logs/simulations/${id_robot}/${id_agent}/${id_stream}.h5'
        filename = os.path.join(self.root,
                               substitute(pattern,
                                          id_agent=id_agent,
                                               id_robot=id_robot,
                                               id_stream=id_stream)) 
        dirname = os.path.dirname(filename)
        if not os.path.exists(dirname):
            os.makedirs(dirname)
        return filename 
    
    
