from . import logger
from bootstrapping_olympics.utils import wrap_script_entry_point
from optparse import OptionParser
import os
import tables
import yaml


def hdf2matlab(pargs):
    usage = 'boot_olympics_hdf2matlab file1.h5 file2.h5 ...'
    parser = OptionParser(usage=usage)
    parser.disable_interspersed_args()

    (_, args) = parser.parse_args(pargs)  

    for filename in args:
        convert_to_matlab(filename)
        
def convert_to_matlab(filename):
    import scipy.io
    logger.info('Reading %s' % filename)
    hdf = tables.openFile(filename)
    stream_name = hdf.root.boot_olympics.streams._v_groups.keys()[0]
    stream = hdf.root.boot_olympics.streams._v_groups[stream_name]
    data = stream.boot_stream[:]
    spec = yaml.load(stream.boot_spec[0])
    remove_none_values(spec)
    
    mf = os.path.splitext(filename)[0] + '.mat'
    variables = dict(stream_name=dict(boot_stream=data, boot_spec=spec))
#    variables = dict(stream_name=dict(boot_stream=data))
#    variables = dict(boot_stream=data)
    logger.info('Writing %s' % mf)
    scipy.io.savemat(mf, variables,
                     oned_as='row',
                     do_compression=True)
    
def remove_none_values(x):
    for k in list(x.keys()):
        if x[k] is None:
            del x[k]
        elif isinstance(x[k], dict):
            remove_none_values(x[k])
              

# TODO: use generic function
def hdf2matlab_main():
    wrap_script_entry_point(hdf2matlab, logger)
    
if __name__ == '__main__':
    hdf2matlab_main()
