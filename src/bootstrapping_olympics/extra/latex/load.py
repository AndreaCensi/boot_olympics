""" Loads reports from disk """
from . import get_sets_dir
import cPickle
import os


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

    with open(filename, 'rb') as f:
        return cPickle.load(f)
        
