import os
import warnings
import zlib

from contracts import contract

from bootstrapping_olympics import BootSpec, logger
from bootstrapping_olympics.utils import (warn_good_identifier,
    warn_good_filename, copy_from, yaml_dump, make_sure_dir_exists)
import numpy as np

from . import tables
from hdflog.hdflogwriter import PGHDFLogWriter
from blocks.interface import Sink
from contracts.utils import check_isinstance
from blocks.library.timed.checks import check_timed_named


warnings.filterwarnings('ignore', category=tables.NaturalNameWarning)


class HDFLogWriter2(Sink):

    def __init__(self, filename, id_stream, boot_spec):
        warn_good_filename(filename)
        warn_good_identifier(id_stream)
        make_sure_dir_exists(filename)
        assert isinstance(boot_spec, BootSpec)
        self.writer = PGHDFLogWriter(filename, compress=True, 
                                     complevel=9, complib='zlib')
        self.boot_spec = boot_spec
        
        self.write_info(id_stream, boot_spec)

    def put(self, value, block=True, timeout=None):  # @UnusedVariable
        check_timed_named(value)
        timestamp, (signal, ob) = value
        
        if signal in ['observations', 'commands']:
            check_isinstance(signal, ob)
            self.writer.log_signal(self, timestamp, signal, value)
        elif signal in ['extra']:
            # this gets dumped to yaml and compressed
            assert isinstance(value, dict)
            extras = yaml_dump(value)
            extras_gz = compress(extras)
#             self.writer.log_signal(self, timestamp, signal, extras_gz)
            self.extra.append(extras_gz)
        else:
            msg = 'Uknown logging signal %r.' % signal
            raise ValueError(msg)
        
#     @contract(observations='array', extra='dict')
#     def push_observations(self, observations, extra={}):
#         if self.table is None:
#             self.table_dtype = remove_fields(observations.dtype, ['extra'])
#             self.create_table(self.table_dtype)
#             # TODO: check that the boot_spec is satisfied
# 
#         # TODO: what about extra?
#         row = np.zeros((), self.table_dtype)
#         copy_from(row, observations)
# 
#         row = row.copy().reshape((1,))
#         self.table.append(row)
# 
#         # ratio = 100.0 * len(extras_gz) / len(extras)
#         # print('Compressed %.1f%%' % (ratio))
#         self.extra.append(extras_gz)

    def write_info(self, id_stream, boot_spec):
        group = self.writer.get_group()
        
        structure = boot_spec.to_yaml()
        yaml_spec = yaml_dump(structure)

        filters_text = tables.Filters(complevel=1, shuffle=False,
                                      fletcher32=False, complib='zlib')

        
        self.extra = self.hf.createVLArray(group, 'extra',
                                           tables.VLStringAtom(),
                                           filters=filters_text)

        boot_spec_table = self.hf.createVLArray(group, 'boot_spec',
                                            tables.VLStringAtom(),
                                            filters=filters_text)
        boot_spec_table.append(yaml_spec)

# 
#     def cleanup(self):
#         """ There was an error; this is not a valid log file; delete. """
#         self.hf.close()
#         if os.path.exists(self.tmp_filename):
#             os.unlink(self.tmp_filename)

    def close(self):
        self.writer.close()
#         if self.table is None:
#             logger.error('No data given for writing; deleting tmp file.')
#             self.hf.close()
#             os.unlink(self.tmp_filename)
#         else:
#             self.hf.close()
#             if os.path.exists(self.filename):
#                 os.unlink(self.filename)
#             os.rename(self.tmp_filename, self.filename)


def remove_fields(dt, fields_to_remove):
    dt2 = []
    for name in dt.names:
        if not name in fields_to_remove:
            dt2.append((name, dt[name]))
    return np.dtype(dt2)


def compress(s):
    ''' Compresses using only one thread. '''
    encoder = zlib.compressobj()
    data = encoder.compress(s)
    data = data + encoder.flush()
    return data


def compress_multi(s):
    ''' Compresses using possibly multiple threads. '''
    return zlib.compress(s)
