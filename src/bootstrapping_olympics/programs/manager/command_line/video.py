from . import check_mandatory, check_no_spurious, OptionParser, declare_command
from ..meat import create_video


@declare_command('video', 'video -a <agent> -r <robot> -e <episode> [-z zoom]')
def cmd_video(data_central, argv):
    '''Creates the video  for a given agent and log. '''
    parser = OptionParser(prog='video', usage=cmd_video.short_usage)
    parser.disable_interspersed_args()
    parser.add_option("-r", "--robot", dest='robot', help="Robot ID")
    parser.add_option("-a", "--agent", dest='agent', help="Agent ID",
                      default="")
    parser.add_option("-e", "--episode", dest='id_episode', help="Episode ID",
                      default="")
#    parser.add_option("-z", "--zoom", dest='zoom', type='float',
#                      default=0,
#                       help="Zoom in m (0: whole map) [%default]")
    parser.add_option("-m", "--model", dest='model', default='boot_log2movie',
                      help="Procgraph model used for visualization. "
                            "[%default]")

    (options, args) = parser.parse_args(argv)

    check_no_spurious(args)
    check_mandatory(options, ['robot'])

    create_video(data_central=data_central,
              id_agent=options.agent,
              id_robot=options.robot,
              id_episode=options.id_episode,
              #zoom=options.zoom,
              model=options.model)

