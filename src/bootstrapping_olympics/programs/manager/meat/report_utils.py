from bootstrapping_olympics.utils.dates import isodate
from . import logger
import cPickle as pickle
import os


def save_report(data_central, report, filename, resources_dir=None,
                save_pickle=False):
    """ filename.html """
    report.to_html(filename, resources_dir=resources_dir)
    report.text('report_date', isodate()) # TODO: add other stuff

    logger.info('Writing to %r.' % filename)

    if save_pickle:
        pickle_name = os.path.splitext(filename)[0] + '.pickle'
        logger.info('Saving to pickle %s.' % pickle_name)
        with open(pickle_name, 'w') as f:
            pickle.dump(report, f) 

    ds = data_central.get_dir_structure()
    ds.file_is_done(filename, desc="%s" % report.nid)
