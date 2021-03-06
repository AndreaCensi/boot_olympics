from . import logger
from ...utils import wrap_script_entry_point, natsorted
from bootstrapping_olympics import get_boot_config
from optparse import OptionParser
import os

usage = """

    This program writes a simple summary of all the configuration available.

    boot_olympics_print_config [-d <config directory>] -o outputdir
"""


def boot_olympics_print_config():
    parser = OptionParser()
    parser.add_option("-o", "--outdir", dest='outdir',
                      help="Output directory")
    parser.add_option("-d", dest="directory",
                      help="base directory for configuration", metavar="FILE")
    (options, args) = parser.parse_args()

    if args:
        raise Exception('Spurious arguments')
    if options.outdir is None:
        raise Exception('Please pass --outdir.')

    print_configuration(options.directory, options.outdir)


def print_configuration(directory, outdir):
    from reprep import Report
    bo_config = get_boot_config()


    bo_config.load(directory)

    def write_report(r):
        out = os.path.join(outdir, '%s.html' % r.id)
        rd = os.path.join(outdir, 'images')
        logger.info('Writing to %r' % out)
        r.to_html(out, resources_dir=rd)

#    tasks = BootOlympicsConfig.tasks
#    r = Report('tasks')
#    create_generic_table(r, 'configuration', tasks, ['desc', 'code'])
#    write_report(r)

    agents = bo_config.agents
    r = Report('agents')
    create_generic_table(r, 'configuration', agents, ['desc', 'code'])
    write_report(r)

    robots = bo_config.robots
    r = Report('robots')
    create_generic_table(r, 'configuration', robots, ['desc', 'ros-node'])
    write_report(r)

    nuisances = bo_config.nuisances
    r = Report('nuisances')
    create_generic_table(r, 'configuration', nuisances, ['desc', 'code'])
    write_report(r)

#    events = BootOlympicsConfig.events
#    r = Report('events')
#    create_generic_table(r, 'configuration', events,
#                         ['desc', 'tasks', 'agents', 'robots'])
#    write_report(r)


def create_generic_table(r, nid, name2entry, cols, caption=None):
    names = natsorted(name2entry.keys())
    if names:
        table = []
        for name in names:
            c = name2entry[name]
            row = []
            for col in cols:
                row.append(c[col])
            table.append(row)
        r.table(nid, table,
                cols=cols, rows=names, caption=caption)
    else:
        logger.warn('warn', 'Empty %r table' % nid)


def main():
    wrap_script_entry_point(boot_olympics_print_config, logger)

if __name__ == '__main__':
    main()
