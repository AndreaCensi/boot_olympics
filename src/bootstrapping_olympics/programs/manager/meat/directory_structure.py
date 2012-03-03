from . import logger
from bootstrapping_olympics import (BootOlympicsConfig, BootOlympicsConstants,
    LogsFormat)
from bootstrapping_olympics.utils import (check_contained, expand_environment,
    substitute, mkdirs_thread_safe, warn_good_identifier, warn_good_filename)
import os
import tempfile


class DirectoryStructure:
    ''' 
        This class knows how to organize the output of the various programs
        on the filesystem.
    '''
    DEFAULT_ROOT = '.'

    DIR_REPORTS = 'reports'
    DIR_VIDEOS = 'videos'
    DIR_CONFIG = 'config'
    DIR_LOGS = 'logs'
    DIR_STORAGE = 'storage'
    DIR_STATES = 'states'

    # XXX: use generic separator
    pattern_simulation = 'simulations/${id_robot}/${id_agent}/'
    pattern_report = '${id_robot}-${id_agent}-${phase}.html'
    pattern_report_rd = 'resources'
    pattern_video = ('${id_robot}-${id_agent}-${id_episode}')

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
        return os.path.join(self.root, DirectoryStructure.DIR_LOGS,
                            'simulations')

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

    def get_simlog_filename(self, id_agent, id_robot, id_stream,
                                  logs_format=None):

        warn_good_identifier(id_stream)

        if logs_format is None:
            logs_format = self.log_format
        check_contained(logs_format, LogsFormat.formats, 'format')

        pattern = DirectoryStructure.pattern_simulation
        dirname = os.path.join(self.root, DirectoryStructure.DIR_LOGS,
                               substitute(pattern,
                                          id_agent=id_agent,
                                          id_robot=id_robot,
                                          id_stream=id_stream))
        mkdirs_thread_safe(dirname)
        assert os.path.exists(dirname)

        # Add a unique string to the file
        tmpfile = tempfile.NamedTemporaryFile(
                                          suffix='.%s.active' % logs_format,
                                          prefix='%s-' % id_stream,
                                          dir=dirname, delete=False)
        tmpfile.write('To be used')

        tmp_filename = tmpfile.name
        filename = tmp_filename.replace('.active', '')

        tmpfile.close()
        logger.debug('Writing on file %r' % filename)

        warn_good_filename(filename)
        warn_good_filename(tmp_filename)

        return filename

    def get_report_res_dir(self, id_agent, id_robot, id_state, phase):
        ''' Returns the directory for the resources of the report. '''
        pattern = DirectoryStructure.pattern_report_rd
        dirname = os.path.join(self.root, DirectoryStructure.DIR_REPORTS,
                               substitute(pattern,
                                          phase=phase,
                                          id_agent=id_agent,
                                          id_robot=id_robot,
                                          id_state=id_state))
        mkdirs_thread_safe(dirname)
        assert os.path.exists(dirname)
        return dirname

    def get_report_filename(self, id_agent, id_robot, id_state, phase):
        pattern = DirectoryStructure.pattern_report
        filename = os.path.join(self.root, DirectoryStructure.DIR_REPORTS,
                               substitute(pattern,
                                          phase=phase,
                                          id_agent=id_agent,
                                          id_robot=id_robot,
                                          id_state=id_state))
        mkdirs_thread_safe(os.path.dirname(filename))
        return filename

    def get_video_basename(self, id_robot, id_agent, id_episode):
        pattern = DirectoryStructure.pattern_video

        if not id_episode:
            id_episode = "alleps"
        if not id_agent:
            id_agent = "allags"

        filename = os.path.join(self.root, DirectoryStructure.DIR_VIDEOS,
                               substitute(pattern,
                                          id_robot=id_robot,
                                          id_agent=id_agent,
                                          id_episode=id_episode))
        warn_good_filename(filename)
        return filename

    def file_is_done(self, filename_or_basename, desc=None):
        """ 
            Notifies that some file is done writing. 
            Used to create a list of recent files that are done.
        """
        # TODO: add implementation of notification





