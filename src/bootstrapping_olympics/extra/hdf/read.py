import numpy as np
from bootstrapping_olympics import logger, get_observations_dtype
from . import load_extra, tables, spec_from_group


__all__ = ['hdf_read']


def hdf_read(filename, id_stream, boot_spec=None, read_extra=False,
              only_episodes=None):
    f = tables.openFile(filename)
    # logger.info("opening file %r" % filename)
    try:
        # TODO: check table exists
        stream_group = f.root.boot_olympics.streams._v_children[id_stream]
        table = stream_group.boot_stream
        extra = stream_group.extra
        if boot_spec is None:
            boot_spec = spec_from_group(stream_group)

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
                if x == 'extra':
                    continue
                observations[x].flat = row[x].flat  # FIXME Strange strange

            if read_extra:
                observations['extra'] = load_extra(extra, i)

            else:
                observations['extra'] = {}
            yield observations
    finally:
        f.close()



