from .. import for_all_pairs
from bootstrapping_olympics.configuration import DirectoryStructure
from bootstrapping_olympics.logs import BootStream, LogIndex
from bootstrapping_olympics.logs import LogsFormat
from bootstrapping_olympics.programs.manager.cmd_simulate import run_simulation
from bootstrapping_olympics.utils import isodate_with_secs
from numpy.testing.utils import assert_equal
import numpy as np
import shutil
import tempfile
from bootstrapping_olympics.interfaces.agent import UnsupportedSpec

@for_all_pairs
def check_writing_logs(id_agent, agent, id_robot, robot):
    try:
        agent.init(robot.get_spec())
    except UnsupportedSpec:
        return

    root = tempfile.mkdtemp()
    ds = DirectoryStructure(root)
    id_stream = isodate_with_secs()
    filename = ds.get_simlog_hdf_filename(id_agent, id_robot, id_stream) # XXX
    
    written = []
    written_extra = []
    
    logs_format = LogsFormat.get_reader_for(filename)
    with logs_format.write_stream(filename=filename,
                                  id_stream=id_stream,
                                  boot_spec=robot.get_spec()) as writer:
        for observations in run_simulation(id_robot, robot, agent, 3, 1000):
            print('observation timestamp %s' % observations['timestamp'])
            extra = {'random_number': np.random.randint(1)}
            writer.push_observations(observations, extra)
            written_extra.append(extra)
            written.append(observations)
            
    logdirs = ds.get_log_directories()
    index = LogIndex()
    for logdir in logdirs:
        index.index(logdir)

    # now use the cached version        
    index = LogIndex()
    for logdir in logdirs:
        index.index(logdir)
        
    assert index.has_streams_for_robot(id_robot)
    streams = index.get_streams_for_robot(id_robot)
    assert len(streams) == 1
    stream = streams[0]
    assert isinstance(stream, BootStream)
    
    assert stream.spec == robot.get_spec()

    read_back = []
    read_back_extra = []
    for observations2 in stream.read():
        read_back_extra.append(observations2['extra'])
        read_back.append(observations2)

    if len(read_back) != len(written):
        raise Exception('Written %d, read back %d.' % 
                        (len(written), len(read_back)))
    
    for i in range(len(read_back)):
        a = written[i]
        b = read_back[i]
        
        fields = set(a.dtype.names) or set(b.dtype.names)
        fields.remove('extra')
        for field in fields:
            assert_equal(a[field], b[field])

    for i in range(len(read_back)):
        a = written_extra[i]
        b = read_back_extra[i]
        assert_equal(a, b)
        
    shutil.rmtree(root)
