""" Loads reports from disk """

from . import get_sets_dir
import cPickle
import os


def get_predict_corr(id_set, agent, robot):
    report_predict = load_report_phase(id_set, agent, robot, 'predict')
    #y_dot = report_predict['y_dot']
    #sys.stderr.write(y_dot.format_tree())
    R = report_predict['y_dot/R'].raw_data 
    # R = np.cos(np.linspace(0, np.pi * 2, 240))
    return R


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
        
