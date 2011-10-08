from ... import BootSpec
from ...utils import yaml_dump
from contracts import contract
import numpy as np
import tables
import warnings

warnings.filterwarnings('ignore', category=tables.NaturalNameWarning)


class HDFLogWriter():
    
    def __init__(self, filename, id_stream, boot_spec):
        assert isinstance(boot_spec, BootSpec)
        self.hf = tables.openFile(filename, 'w')
        self.id_stream = id_stream
        self.table = None
        self.boot_spec = boot_spec
        
    @contract(observations='array')
    def push_observations(self, observations, extra={}):
        if self.table is None:
            self.table_dtype = remove_fields(observations.dtype, ['extra'])
            self.create_table(self.table_dtype)
            
        # TODO: what about extra?
        row = np.zeros((1), self.table_dtype)
        
        for name in row.dtype.names:
            row[name] = observations[name].flat
        
        row = row.copy().reshape((1,)) 
        self.table.append(row)
        self.extra.append(yaml_dump(extra))
        
    def create_table(self, dtype):                
        filters = tables.Filters(complevel=9, complib='zlib',
                                 fletcher32=True)
        group = '/boot_olympics/streams/%s' % self.id_stream
        self.table = self.hf.createTable(
                        where=group,
                        name='boot_stream',
                        description=dtype,
                        filters=filters,
                        createparents=True
                    )
        self.table.attrs['boot_spec'] = yaml_dump(self.boot_spec.to_yaml())

        self.extra = self.hf.createVLArray(group, 'extra',
                                           tables.VLUnicodeAtom(),
                                           filters=filters)

    def close(self):
        self.table.flush()
        self.hf.close() 


def remove_fields(dt, fields_to_remove):
    dt2 = []
    for name in dt.names:
        if not name in fields_to_remove:
            dt2.append((name, dt[name]))
    return np.dtype(dt2)
