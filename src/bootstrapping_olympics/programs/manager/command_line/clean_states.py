from . import check_no_spurious, logger, OptionParser, declare_command


@declare_command('clean-states', 'clean-states')
def cmd_clean_states(data_central, argv):
    ''' Cleans agents states. '''

