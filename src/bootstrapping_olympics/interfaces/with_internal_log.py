from bootstrapping_olympics import logger
from contracts import contract
import time

__all__ = ['BootWithInternalLog']

class BootWithInternalLog(object):
    """ 
        Subclassing this class gives the object the capability
        of calling self.info, self.error, etc. and have their
        logging memorized.
    """
        
    def _init_log(self):
        self.log_lines = []  # log records
        self.children = {}    
        name = self.__class__.__name__  # don't call str() yet
        self.name = name
        self.set_log_output(True)
        
    def _check_inited(self):
        """ Make sure that we inititalized the log system.
            We don't count on a constructor being called. """
        if not 'name' in self.__dict__:
            self._init_log()
        
    @contract(name='str')
    def set_name_for_log(self, name):
        self._check_inited()
        self.name = name
        
        # update its names
        for id_child, child in self.children.items():
            its_name = self.name + '>' + id_child
            child.set_name_for_log(its_name)
            
    @contract(id_child='str')
    def log_add_child(self, id_child, child):
        self._check_inited()
        if not isinstance(child, BootWithInternalLog):
            msg = 'Tried to add child of type %r' % type(child)
            self.error(msg)
            return
        self.children[id_child] = child
        its_name = self.name + '>' + id_child
        child.set_name_for_log(its_name)
    
    @contract(enable='bool')
    def set_log_output(self, enable):
        self._check_inited()
        """ 
            Enable or disable instantaneous on-screen logging.
            If disabled, things are still memorized.     
        """
        self.log_output_enabled = enable
    
    def _save_and_write(self, s, level):
        record = BootLogRecord(name=self.name, timestamp=time.time(), string=s,
                               level=level)
        self.log_lines.append(record)
        if self.log_output_enabled:
            record.write_to_logger(logger)
        
    @contract(s='str')
    def info(self, s):
        """ Logs a string; saves it for visualization. """
        self._check_inited()
        self._save_and_write(s, 'info')
        
    @contract(s='str')
    def debug(self, s):
        self._check_inited()
        self._save_and_write(s, 'debug')
    
    @contract(s='str')
    def error(self, s):
        self._check_inited()
        self._save_and_write(s, 'error')

    @contract(s='str')
    def warn(self, s):
        self._check_inited()
        self._save_and_write(s, 'warn')
    
    def get_log_lines(self):
        """ Returns a list of LogRecords """
        self._check_inited()
        lines = list(self.log_lines)
        for child in self.children.values():
            lines.extend(child.get_log_lines())
        lines.sort(key=lambda x: x.timestamp)
        return lines
    
    def get_raw_log_lines(self):
        """ Returns a list of strings """
        self._check_inited()
        raw = map(BootLogRecord.__str__, self.get_log_lines())
        return raw


class BootLogRecord(object):
    
    def __init__(self, name, timestamp, string, level):
        self.name = name
        self.timestamp = timestamp
        self.string = string
        self.level = level
    
        levels = ['info', 'error', 'debug', 'warn']
        if not level in levels:
            msg = 'Got %r, expected %r.' % (level, levels)
            raise ValueError(msg)
        
    def __str__(self):
        return '%s: %s' % (self.name, self.string)
        
    def write_to_logger(self, logger):
        s = self.__str__()
        level = self.level
        if level == 'info':
            logger.info(s)
        elif level == 'error':
            logger.error(s)
        elif level == 'debug':
            logger.debug(s)
        elif level == 'warn':
            logger.warn(s)
        else:
            assert False
