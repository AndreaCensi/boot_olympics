'''Some functions to help in writing experiments scripts'''

from optparse import OptionParser
import contracts
import os
import shutil
from bootstrapping_olympics.programs.manager.batch import default_expl_videos
from . import logger
from bootstrapping_olympics.programs.manager.meat.data_central import DataCentral
from bootstrapping_olympics.programs.manager.batch.batch_learn import TaskRegister


def experiment_explore_learn_main(proj_root,
                                  explorer, agents, robots,
                                  args):
    parser = OptionParser()
    parser.disable_interspersed_args()
    parser.add_option("--reset", default=False, action='store_true',
                      help="Reset the state of the agents.")
    parser.add_option("--resimulate", default=False, action='store_true',
                      help="Resimulates the logs.")
    parser.add_option("--republish", default=False, action='store_true',
                      help="Cleans the reports.")
    parser.add_option("--num_ep_expl", type='int', default=10,
                      help="Number of episodes to simulate [%default]")
    parser.add_option("--num_ep_expl_v", type='int', default=6,
                      help="Number of videos of exploration to create"
                            " [%default]")
    parser.add_option("--num_ep_serv", type='int', default=0,
                      help="Number of servoing episodes to simulate "
                      "[%default]")
    parser.add_option("--num_ep_serv_v", type='int', default=0,
                      help="Number of videos for servoing. [%default]")

    parser.add_option("--episode_len", type='float', default=30,
                      help="Maximum len of episode (seconds) [%default]")
    parser.add_option("--contracts", default=False, action='store_true',
                      help="Slower, more checks.")
    parser.add_option("--only", default=None,
                      help="Only do things for this robot.")
    (options, args) = parser.parse_args(args)

    if not options.contracts:
        logger.info('Disabling PyContracts for maximum speed'
                    ' (use --contracts to enable).')
        contracts.disable_all()

    from compmake import compmake_console, batch_command #@UnresolvedImport
    data_central = DataCentral(proj_root)

    experiment_explore_learn_compmake(data_central, explorer, agents, robots,
                         episode_len=options.episode_len,
                         num_ep_expl=options.num_ep_expl,
                         num_ep_expl_v=options.num_ep_expl_v,
                         num_ep_serv=options.num_ep_serv,
                         num_ep_serv_v=options.num_ep_serv_v,
                         reset=options.reset
                         )

    if options.only:
        id_robot = options.only

    if options.reset:
        if options.only:
            batch_command('clean learn*X*'.replace('X', id_robot))
        else:
            batch_command('clean learn*')

    if options.republish:
        logger.info('Removing previous logs.')
        dirname = os.path.join(proj_root, 'reports', 'learn')
        if os.path.exists(dirname):
            shutil.rmtree(dirname)

        batch_command('clean publish*')

    if options.resimulate:
        logger.info('Removing previous logs.')
        if options.only:
            dirname = os.path.join(proj_root, 'logs', 'simulations', id_robot)
        else:
            dirname = os.path.join(proj_root, 'logs', 'simulations')

        if os.path.exists(dirname):
            shutil.rmtree(dirname)

        batch_command('clean simulate*')

    if options.only:
        id_robot = options.only
        logger.info('Only doing things for robot %r.' % id_robot)
        cmd1 = 'make sim*X* pub*X* '.replace('X', id_robot)
        batch_command(cmd1)
        if options.write_extra:
            cmd2 = 'make video-exploration-X-ep0000-*'.replace('X', id_robot)
            batch_command(cmd2)

    compmake_console()


def experiment_explore_learn_compmake(data_central, **kwargs):
    """
        
        
        :param:publish_progress: publish results after each tranche is learned

    """

    tr = TaskRegister(data_central)

    tr.main(**kwargs)

