from blocks.library.timed.checks import check_timed_named
from boot_manager import BootStream, DirectoryStructure, LogIndex, LogsFormat
from boot_manager.meat.m_run_simulation import run_simulation
from bootstrapping_olympics import ExploringAgent, UnsupportedSpec
from bootstrapping_olympics.unittests import for_all_pairs
from bootstrapping_olympics.utils import (assert_allclose, 
    unique_timestamp_string)
from comptests import Skipped
from contracts import describe_value
from numpy.testing.utils import assert_equal
import numpy as np
import shutil
import tempfile




@for_all_pairs
def check_logs_writing(id_agent, agent, id_robot, robot):
    if not isinstance(agent, ExploringAgent):
        return Skipped('agent not ExploringAgent')

    try:
        agent.init(robot.get_spec())
    except UnsupportedSpec:
        return Skipped('UnsupportedSpec')

    root = tempfile.mkdtemp()
    ds = DirectoryStructure(root)
    id_stream = unique_timestamp_string()
    filename = ds.get_simlog_filename(id_agent, id_robot, id_stream)
    id_episode = id_stream
    written = []
    written_extra = []

    logs_format = LogsFormat.get_reader_for(filename)
    with logs_format.write_stream(filename=filename,
                                  id_stream=id_stream,
                                  boot_spec=robot.get_spec(),
                                  id_agent=id_agent,
                                  id_robot=id_robot) as writer:

        writer.reset()                    
        for x in run_simulation(
                                           robot=robot,
                                           
                                           agent=agent,
                                           max_observations=3, max_time=1000,
                                           check_valid_values=True):
            
            check_timed_named(x)
            timestamp, (signal, value) = x
            
            writer.put(x)
            
            if signal == 'observations':
                written.append(x)
            
            if signal == 'observations':
                extra = {'random_number': np.random.randint(1)}
                
                m = (timestamp, ('extra', extra))
                writer.put(m)
                
                m = (timestamp, ('id_episode', id_episode))
                writer.put(m)
#                 written.append(m)
                        
            written_extra.append(extra)
            

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

    assert stream.get_spec() == robot.get_spec()

    # only read back observations
    read_back = []
    read_back_extra = []
    for x in stream.read(read_extra=True):
        check_timed_named(x)
        (timestamp, (signal, value))  = x
        if signal == 'extra':
            read_back_extra.append(value)
        elif signal == 'observations':
            read_back.append(x)
        else:
            pass

    if len(read_back) != len(written):
        raise Exception('Written %d, read back %d.' % 
                        (len(written), len(read_back)))

    for i in range(len(read_back)):
        a = written[i]
        b = read_back[i]
        try:
            assert a[0] == b[0]
            assert a[1][0] == b[1][0] # signal
            assert_allclose(a[1][1], b[1][1])
        except:
            print('found mismatch at i = %d' % i)
            print('written = %s' % describe_value(a))
            print('read = %s' % describe_value(b))
            raise
#         fields = set(a.dtype.names) or set(b.dtype.names)
#         fields.remove('extra')
#         for field in fields:
#             assert_equal(a[field], b[field])

    for i in range(len(read_back)):
        a = written_extra[i]
        b = read_back_extra[i]
        assert_equal(a, b)

    shutil.rmtree(root)
