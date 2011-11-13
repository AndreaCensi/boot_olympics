from . import contract, logger
from bootstrapping_olympics.interfaces.boot_spec import BootSpec
from bootstrapping_olympics.logs.ros.ros_conversions import observations2ros
import os

class BagLogWriter():
    # TODO: compress the bag
        
    def __init__(self, filename, id_stream, boot_spec):
        from ros import rosbag #@UnresolvedImport
        assert isinstance(boot_spec, BootSpec)
        self.filename = filename
        # XXX: check that we are not given the same filename
        self.table = None
        self.boot_spec = boot_spec
        self.topic = id_stream
        self.tmp_filename = filename + '.active'
        self.bag = rosbag.Bag(self.tmp_filename, 'w',
                              compression=rosbag.Compression.BZ2)
        self.num = 0
        
    @contract(observations='array')
    def push_observations(self, observations, extra={}):
        from rospy import rostime #@UnresolvedImport
        msg = observations2ros(self.boot_spec, observations)
        
        t = rostime.Time.from_sec(observations['timestamp'])
        self.bag.write(self.topic, msg, t)
        self.num += 1
        

    def close(self): 
        if self.num == 0:
            logger.error('No data given for writing; deleting tmp file.')
            self.bag.close()
            os.unlink(self.tmp_filename)
        else:
            self.bag.close() 
            if os.path.exists(self.filename):
                os.unlink(self.filename)
            os.rename(self.tmp_filename, self.filename)
