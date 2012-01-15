from . import  OptionParser, declare_command
from ..batch import batch_process_manager


@declare_command('batch', 'batch [set1,set2]')
def cmd_batch(data_central, argv):
    '''Runs the learning for a given agent and log. '''
    parser = OptionParser(prog='batch', usage=cmd_batch.short_usage)
    parser.disable_interspersed_args()

    (_, args) = parser.parse_args(argv)

    which_sets = args

    batch_process_manager(data_central=data_central,
                          which_sets=which_sets)

