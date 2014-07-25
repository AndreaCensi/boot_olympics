from procgraph.core.registrar_other import simple_block
import numpy as np
from procgraph.core.constants import REQUIRED

@simple_block
def extract_sensels(bd):
    return np.array(bd['observations'])


@simple_block
def extract_commands(bd):
    return np.array(bd['commands'])


@simple_block
def extract_from_extra(extra, fieldname=REQUIRED):
    if not fieldname in extra:
        msg = 'No %r field in %s.' % (fieldname, list(extra))
        raise ValueError(msg)
    return extra[fieldname]
