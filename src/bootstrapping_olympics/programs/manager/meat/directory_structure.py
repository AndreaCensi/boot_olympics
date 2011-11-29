from . import logger
from ....configuration import  BootOlympicsConfig
from ....utils import expand_environment, substitute
import os
import tempfile
from bootstrapping_olympics.constants import BootOlympicsConstants
from bootstrapping_olympics.logs.logs_format import LogsFormat
from bootstrapping_olympics.utils.dict_utils import check_contained
from bootstrapping_olympics.utils.filesystem_utils import mkdirs_thread_safe

class DirectoryStructure:
    
    DEFAULT_ROOT = '~/boot_olympics/'
    
    DIR_REPORTS = 'reports'
    DIR_VIDEOS = 'videos'
    DIR_CONFIG = 'config'
    DIR_LOGS = 'logs'
    DIR_STORAGE = 'storage'
    DIR_STATES = 'states'
    
    def __init__(self, root=None):
        if root is None: 
            root = DirectoryStructure.DEFAULT_ROOT
        self.root = expand_environment(root)
        
        if not os.path.exists(self.root):
            msg = 'The root directory %r does not exist.' % self.root
            raise Exception(msg)
        
        self.log_format = BootOlympicsConstants.DEFAULT_LOG_FORMAT
        
    def set_log_format(self, log_format):
        ''' Sets the log format to write logs with. '''
        check_contained(log_format, LogsFormat.formats, 'format')
        self.log_format = log_format
     
    def get_config_directories(self):
        dirs = []
        dirs.append(BootOlympicsConfig.get_default_dir())
        dirs.append(os.path.join(self.root, DirectoryStructure.DIR_CONFIG))
        return dirs
         
    def get_simulated_logs_dir(self):
        ''' Returns the main directory where simulated logs are placed. '''
        return os.path.join(self.root, DirectoryStructure.DIR_LOGS, 'simulations')
    
    def get_storage_dir(self):
        ''' Returns a directory where intermediate results can be placed. '''
        return os.path.join(self.root, DirectoryStructure.DIR_STORAGE)
    
    def get_log_directories(self):
        ''' Returns a list of the directories where to look for logs. '''
        dirs = []
        dirs.append(os.path.join(self.root, DirectoryStructure.DIR_LOGS))
        # TODO: additional
        return dirs
    
    def get_state_db_directory(self):
        return os.path.join(self.root, self.DIR_STATES)

    def get_simlog_filename(self, id_agent, id_robot, id_stream, logs_format=None):
        if logs_format is None:
            logs_format = self.log_format
        check_contained(logs_format, LogsFormat.formats, 'format')
        
        pattern = 'simulations/${id_robot}/${id_agent}/'
        dirname = os.path.join(self.root, DirectoryStructure.DIR_LOGS,
                               substitute(pattern,
                                          id_agent=id_agent,
                                          id_robot=id_robot,
                                          id_stream=id_stream)) 
        mkdirs_thread_safe(dirname)
        assert os.path.exists(dirname)

        tmpfile = tempfile.NamedTemporaryFile(suffix='.%s.active' % logs_format,
                                              prefix=id_stream,
                                              dir=dirname, delete=False)
        tmpfile.write('To be used')
        
        tmp_filename = tmpfile.name
        filename = tmp_filename.replace('.active', '')

        tmpfile.close()
        logger.debug('Writing on file %r' % filename)
        
        return filename 
    
    
    
    def get_report_dir(self, id_agent, id_robot, id_state, phase='learn',
                        add_last=False):
        pattern = '${id_robot}/${id_agent}'
        dirname = os.path.join(self.root, DirectoryStructure.DIR_REPORTS,
                               phase,
                               substitute(pattern,
                                          phase=phase,
                                          id_agent=id_agent,
                                          id_robot=id_robot,
                                          id_state=id_state)) 
        mkdirs_thread_safe(dirname)
        assert os.path.exists(dirname)
      
        if add_last:
            last1 = os.path.join(self.root, DirectoryStructure.DIR_REPORTS,
                                 phase,
                                   substitute(pattern,
                                              id_agent=id_agent,
                                              id_robot=id_robot,
                                              id_state='last')) 
            last2 = os.path.join(self.root, DirectoryStructure.DIR_REPORTS,
                                 'last') 
            
            for l in [last1, last2]:
                if os.path.exists(l):
                    os.unlink(l)
                os.symlink(dirname, l)

        return dirname 
        
    def get_video_basename(self, id_robot, id_agent, id_episode):
        pattern = '${id_robot}/${id_episode}'
        filename = os.path.join(self.root, DirectoryStructure.DIR_VIDEOS,
                               substitute(pattern,
                                          id_robot=id_robot,
                                          id_episode=id_episode))
        return filename
