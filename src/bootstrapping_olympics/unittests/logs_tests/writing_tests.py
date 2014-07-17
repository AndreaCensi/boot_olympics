from bootstrapping_olympics import (BootStream, ExploringAgent, LogIndex, 
    LogsFormat, ObsKeeper, UnsupportedSpec)
from bootstrapping_olympics.programs.manager import DirectoryStructure # XXX
from bootstrapping_olympics.programs.manager import run_simulation
from bootstrapping_olympics.unittests import for_all_pairs
from bootstrapping_olympics.utils import unique_timestamp_string
from comptests.results import Skipped
from numpy.testing.utils import assert_equal
import numpy as np
import shutil
import tempfile
from contracts.utils import check_isinstance




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

    written = []
    written_extra = []

    logs_format = LogsFormat.get_reader_for(filename)
    with logs_format.write_stream(filename=filename,
                                  id_stream=id_stream,
                                  boot_spec=robot.get_spec()) as writer:
        
        ok = ObsKeeper(boot_spec=robot.get_spec(), id_robot=id_robot)
                    
        for t, bd in run_simulation(id_robot=id_robot,
                                           robot=robot,
                                           id_agent=id_agent,
                                           agent=agent,
                                           max_observations=3, max_time=1000,
                                           check_valid_values=True):
            extra = {'random_number': np.random.randint(1)}            
                            
            bd_array = ok.push(timestamp=t, 
                               observations=bd['observations'],
                               commands=bd['commands'],
                               commands_source=id_agent,
                               id_episode='my_episode',
                               id_world='unknown-world')
                  
            writer.push_observations(bd_array, extra)
    
            written_extra.append(extra)
            written.append(bd_array) # not sure

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

    read_back = []
    read_back_extra = []
    for observations2 in stream.read(read_extra=True):
        read_back_extra.append(observations2['extra'])
        read_back.append(observations2)

    if len(read_back) != len(written):
        raise Exception('Written %d, read back %d.' % 
                        (len(written), len(read_back)))

    for i in range(len(read_back)):
        a = written[i]
        b = read_back[i]
        check_isinstance(a, np.ndarray)
        check_isinstance(b, np.ndarray)

        fields = set(a.dtype.names) or set(b.dtype.names)
        fields.remove('extra')
        for field in fields:
            assert_equal(a[field], b[field])

    for i in range(len(read_back)):
        a = written_extra[i]
        b = read_back_extra[i]
        assert_equal(a, b)

    shutil.rmtree(root)
