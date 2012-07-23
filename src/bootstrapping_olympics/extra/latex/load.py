""" Loads reports from disk """
from . import get_sets_dir
import os
from bootstrapping_olympics.utils.safe_pickle import safe_pickle_load
from bootstrapping_olympics.utils.warn_long_time_exc import warn_long_time


def load_report_phase(id_set, agent, robot, phase):
    basename = '%s/%s/reports/%s-%s-%s' % (get_sets_dir(),
                                           id_set, robot, agent, phase)
    return load_report_file(basename)

    
def load_report_robot(id_set, robot):
    basename = '%s/%s/reports/%s' % (get_sets_dir(), id_set, robot)
    return load_report_file(basename)

        
def load_report_file(basename):
    filename = basename + '.pickle' 
    if not os.path.exists(filename):
        msg = 'Report %s not found' % filename
        raise Exception(msg)

    with warn_long_time(1, 'loading report %r' % basename) as moreinfo:
        moreinfo['size'] = os.stat(filename).st_size
        return safe_pickle_load(filename)
    
