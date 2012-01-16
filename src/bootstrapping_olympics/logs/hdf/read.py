from . import logger, np, tables, spec_from_table
from ... import get_observations_dtype
from ...utils import yaml_load
import time

__all__ = ['hdf_read']


def hdf_read(filename, id_stream, boot_spec=None, read_extra=False,
              only_episodes=None):
    f = tables.openFile(filename)
    try:
        # TODO: check table exists
        table = f.root.boot_olympics.streams._v_children[id_stream].boot_stream
        if boot_spec is None: boot_spec = spec_from_table(table)
        extra = f.root.boot_olympics.streams._v_children[id_stream].extra
        n = len(table)
        n_extra = len(extra)
        if n != n_extra:
            msg = ('In stream %s:%s I see %d observations, but only %d extra.'
                   % (filename, id_stream, n, n_extra))
            logger.warn(msg)
        n = min(n, n_extra)
        dtype = get_observations_dtype(boot_spec)
        for i in range(n):
            row = table[i]

            id_episode = row['id_episode'].item()
            if only_episodes and not id_episode in only_episodes:
                continue

            observations = np.zeros((), dtype)
            for x in dtype.names:
                if x == 'extra': continue
                observations[x].flat = row[x].flat # FIXME Strange strange
            if read_extra:
                extra_string = str(extra[i])
                assert isinstance(extra_string, str)


                t0 = time.clock()

                observations['extra'] = yaml_load(extra_string)

                if False:
                    logger.debug('read string of len %s in %s' %
                                 (len(extra_string), time.clock() - t0))
            else:
                observations['extra'] = {}
            yield observations
    finally:
        f.close()

