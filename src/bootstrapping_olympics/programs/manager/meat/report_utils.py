from bootstrapping_olympics import logger
from bootstrapping_olympics.utils import (isodate, safe_pickle_dump,
    warn_long_time_writing, warn_long_time_reading)
from reprep.output import report_from_hdf
import os


def save_report(data_central, report, filename, resources_dir=None,
                save_pickle=False, save_hdf=True,
                check_hdf_written_correctly=True):
    """ filename.html """
    
    report.text('report_date', isodate())  # TODO: add other stuff
    
    ds = data_central.get_dir_structure()
    
    report.to_html(filename, resources_dir=resources_dir)
    ds.file_is_done(filename)
    
    if save_pickle:
        pickle_name = os.path.splitext(filename)[0] + '.pickle'
        
        with warn_long_time_writing(pickle_name):
            safe_pickle_dump(report, pickle_name, protocol=2)

        ds.file_is_done(pickle_name)


    if save_hdf:
        hdf_name = os.path.splitext(filename)[0] + '.rr1.h5'
        
        with warn_long_time_writing(hdf_name):
            report.to_hdf(hdf_name)
        
        if check_hdf_written_correctly:
            with warn_long_time_reading(hdf_name, logger=logger):
                r2 = report_from_hdf(hdf_name)

            if report != r2:
                logger.error("Dont match")
                logger.error(report.format_tree())
                logger.error(r2.format_tree())
                raise Exception()
        
        ds.file_is_done(hdf_name)
        
    
    
