import os

from bootstrapping_olympics import logger
from bootstrapping_olympics.logs import LogsFormat

from .main import BOM


class CmdCompact(BOM.get_sub()):
    ''' Compacts the logs into the given directory. '''
    
    cmd = 'compact'

    def define_program_options(self, params):
        params.add_flag('output', short='-o')

    def go(self):
        output = self.get_options().output
        data_central = self.get_parent().get_data_central()
        
        index = data_central.get_log_index()
    
        print('Index contains %d bag files with boot data.' % 
                    len(index.file2streams))
        print('In total, there are %d robots:' % len(index.robots2streams))
    
        for id_robot in index.robots2streams:
            write_robot(index, id_robot, output, skip_existing=True)
    

#
# @declare_command('compact', 'compact -o <directory>')
# def cmd_compact(data_central, argv):
#     ''' Compacts the logs into the given directory. '''
#     parser = OptionParser(prog='list-robots',
#                           usage=cmd_compact.short_usage)
#     parser.disable_interspersed_args()
#     parser.add_option("-o", "--output")
#     (options, args) = parser.parse_args(argv)
#
#     check_no_spurious(args)
#     if options.output is  None:
#         msg = 'Need output option'
#         raise UserError(msg)
#
#     index = data_central.get_log_index()
#
#     print('Index contains %d bag files with boot data.' %
#                 len(index.file2streams))
#     print('In total, there are %d robots:' % len(index.robots2streams))
#
#     for id_robot in index.robots2streams:
#         write_robot(index, id_robot, options.output, skip_existing=True)


def write_robot(index, id_robot, output_dir, skip_existing=True):
    streams = index.robots2streams[id_robot]
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    filename = os.path.join(output_dir, '%s.h5' % id_robot)
    if skip_existing and os.path.exists(filename):
        logger.info('Skipping %s' % filename)
        return
    id_stream = '%s_all' % id_robot
    logger.info('Writing all to %r' % filename)
    
    boot_spec = streams[0].get_spec()
    logs_format = LogsFormat.get_reader_for(filename)
    with logs_format.write_stream(filename=filename,
                                  id_stream=id_stream,
                                  boot_spec=boot_spec) as writer:
        for stream in streams:
            logger.info('- reading stream %s' % stream)
            for observations in stream.read(read_extra=False):
                writer.push_observations(observations)

        



