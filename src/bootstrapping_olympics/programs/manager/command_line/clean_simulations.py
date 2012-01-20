from . import check_no_spurious, logger, OptionParser, declare_command


@declare_command('clean-simulations', 'clean-states')
def cmd_clean_simulations(data_central, argv):
    ''' Cleans all simulations (not real logs) '''

