from .. import getLogger
logger = getLogger(__name__)

try:
    import procgraph
except ImportError as e:
    boot_has_procgraph = False
    procgraph_error = e
    logger.warning('ProcGraph support not available (%s).' % procgraph_error)
else:
    boot_has_procgraph = True
    procgraph_error = 'everything is fine'
    from . import boot_log_reader


