from . import load_configuration, Configuration
from optparse import OptionParser
from reprep import Report
import logging
import os
from .natsort import natsorted

logging.basicConfig();
logger = logging.getLogger("print_config")
logger.setLevel(logging.DEBUG)


usage = """

    This program writes a simple summary of all the configuration available.

    boot_olympics_print_config [-d <config directory>] -o outputdir
""" 


def main():
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
    load_configuration(directory)
     
    def write_report(r):
        out = os.path.join(outdir, '%s.html' % r.id)
        rd = os.path.join(outdir, 'images')
        logger.info('Writing to %r' % out)
        r.to_html(out, resources_dir=rd)
        
    tasks = Configuration.tasks
    r = Report('tasks')
    create_generic_table(r, 'configuration', tasks, ['desc', 'code'])
    write_report(r)
    
    agents = Configuration.agents
    r = Report('agents')
    create_generic_table(r, 'configuration', agents, ['desc', 'code'])
    write_report(r)
    
    robots = Configuration.robots
    r = Report('robots')
    create_generic_table(r, 'configuration', robots, ['desc', 'ros-node'])
    write_report(r)
    
    events = Configuration.events
    r = Report('events')
    create_generic_table(r, 'configuration', events,
                         ['desc', 'tasks', 'agents', 'robots'])
    write_report(r)
    

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
    
if __name__ == '__main__':
    main()

