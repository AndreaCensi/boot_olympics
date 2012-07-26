from . import logger
from bootstrapping_olympics.utils import (isodate, safe_pickle_dump,
    warn_long_time)
from conf_tools.utils import friendly_path
import os
from reprep.output.hdf.hdf_read import report_from_hdf


def save_report(data_central, report, filename, resources_dir=None,
                save_pickle=False, save_hdf=True):
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

    if save_hdf:
        hdf_name = os.path.splitext(filename)[0] + '.rr1.h5'
        
        logger.info('Saving to hdf %s.' % friendly_path(hdf_name))
        
        with warn_long_time(0, 'writing hdf', logger) as moreinfo:
            report.to_hdf(hdf_name)
            moreinfo['size'] = os.stat(pickle_name).st_size

            
        with warn_long_time(0, 'reading hdf', logger) as moreinfo:
            r2 = report_from_hdf(hdf_name)
            moreinfo['size'] = os.stat(hdf_name).st_size
            
        if report != r2:
            logger.error("Dont match")
            logger.error(report.format_tree())
            logger.error(r2.format_tree())
            raise Exception()
        else:
            logger.info("OK match")

    ds = data_central.get_dir_structure()
    ds.file_is_done(filename, desc="%s" % report.nid)
