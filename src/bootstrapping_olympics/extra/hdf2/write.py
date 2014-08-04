from . import tables
from blocks import Sink
from blocks.library.timed.checks import check_timed_named
from blocks.utils import check_reset
from bootstrapping_olympics import BootSpec
from bootstrapping_olympics.utils import (make_sure_dir_exists, 
    warn_good_filename, warn_good_identifier, yaml_dump)
from collections import defaultdict
from contracts.utils import check_isinstance
from hdflog import PGHDFLogWriter
import numpy as np
import warnings



warnings.filterwarnings('ignore', category=tables.NaturalNameWarning)


class HDFLogWriter2(Sink):

    def __init__(self, filename, id_stream, boot_spec, id_agent, id_robot):
        warn_good_filename(filename)
        warn_good_identifier(id_stream)
        make_sure_dir_exists(filename)
        self.filename = filename
        assert isinstance(boot_spec, BootSpec)
        self.boot_spec = boot_spec
        self.id_stream = id_stream
        self.id_agent = id_agent
        self.id_robot = id_robot
        
        self.num_put = defaultdict(lambda: 0)
        
    def reset(self):
        self.writer = PGHDFLogWriter(self.filename, compress=True, 
                                     complevel=9, complib='zlib')
        
        
        self.write_info()

    def put(self, value, block=True, timeout=None):  # @UnusedVariable
        check_reset(self, 'writer')
        
        check_timed_named(value)
        timestamp, (signal, ob) = value
        
        #self.info('received %s %s' % (timestamp, signal))
        
        self.num_put[signal] += 1
        
        if signal in ['observations', 'commands', 'robot_pose']:
            # TODO: check spec
            check_isinstance(ob, np.ndarray)
            self.writer.log_signal(timestamp, signal, ob)
        elif signal in ['id_episode']:
            self.writer.log_short_string(timestamp, signal, ob)
        elif signal in ['extra']:
            # this gets dumped to yaml and compressed
            self.writer.log_compressed_yaml(timestamp, signal, ob)
        else:
            msg = 'Unknown logging signal %r.' % signal
            raise ValueError(msg) 
        
    def end_input(self):
        check_reset(self, 'writer')

        self.close()

    def write_info(self): 
        structure = self.boot_spec.to_yaml()
        
        info = dict(id_robot=self.id_robot,
                    id_agent=self.id_agent,
                    id_stream=self.id_stream,
                    boot_spec=structure) 

        yaml_spec = yaml_dump(info)
        self.writer.log_large_string(0.0, 'boot_info', yaml_spec)
 

    def close(self):
        check_reset(self, 'writer')
        
        self.info('Messages seen: %s' % dict(**self.num_put))
        if not self.num_put:
            msg = 'No messages seen.'
            raise Exception(msg)
        if not 'id_episode' in self.num_put:
            msg = 'Did not lot any id_episode: %s' % dict(**self.num_put)
            raise Exception(msg)
            
        self.writer.finish() 
        
    def cleanup(self):
        check_reset(self, 'writer')
        self.writer.finish()

