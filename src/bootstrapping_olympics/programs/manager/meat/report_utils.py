from . import logger
from bootstrapping_olympics.utils import (isodate, safe_pickle_dump,
    warn_long_time)
from conf_tools.utils import friendly_path
import os


def save_report(data_central, report, filename, resources_dir=None,
                save_pickle=False):
    """ filename.html """
    report.to_html(filename, resources_dir=resources_dir)
    report.text('report_date', isodate()) # TODO: add other stuff

    logger.info('Writing to %r.' % friendly_path(filename))

    if save_pickle:
        pickle_name = os.path.splitext(filename)[0] + '.pickle'
        logger.info('Saving to pickle %s.' % friendly_path(pickle_name))
        with warn_long_time(0, 'writing pickle', logger) as moreinfo:
            safe_pickle_dump(report, pickle_name, protocol=2)
            moreinfo['size'] = os.stat(pickle_name).st_size

    ds = data_central.get_dir_structure()
    ds.file_is_done(filename, desc="%s" % report.nid)
