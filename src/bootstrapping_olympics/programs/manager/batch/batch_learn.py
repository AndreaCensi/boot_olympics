'''Some functions to help in writing experiments scripts'''
from . import (default_expl_videos, default_servo_videos,
    default_servonav_videos, contract)
from .. import (create_video, servo_stats_report, servo_stats_summaries,
    simulate, task_predict, logger, learn_log, publish_once, np, task_servo,
    task_servonav)
from bootstrapping_olympics import UnsupportedSpec
from conf_tools import SemanticMistake
import itertools

try:
    from compmake import comp
except ImportError:
    pass
    # TODO: add message


def batch_jobs1(data_central, **kwargs):
    tr = TaskRegister(data_central)

    tr.main(**kwargs)


class TaskRegister:

    def __init__(self, data_central):
        self.data_central = data_central

        self.deps = {}

    @contract(id_agent='str', K='int')
    def episode_id_exploration(self, id_agent, K):
        return 'ep_expl_%s_%05d' % (id_agent, K)

    @contract(id_agent='str', K='int')
    def episode_id_servoing(self, id_agent, K):
        return 'ep_serv_%s_%05d' % (id_agent, K)

    @contract(id_agent='str', K='int')
    def episode_id_servonav(self, id_agent, K):
        return 'ep_servonav_%s_%05d' % (id_agent, K)

    def agent_has_predictor(self, id_agent):
        agent = self.data_central.get_bo_config().agents.instance(id_agent)
        return hasattr(agent, 'get_predictor')

    def agent_has_servo(self, id_agent):
        agent = self.data_central.get_bo_config().agents.instance(id_agent)
        return hasattr(agent, 'get_servo')

    def get_tranches(self, ids, episodes_per_tranche=10):
        """ Returns a list of list """
        l = []
        num_tranches = int(np.ceil(len(ids) * 1.0 / episodes_per_tranche))
        for t in range(num_tranches):
            e_from = t * episodes_per_tranche
            e_to = min(len(ids), e_from + episodes_per_tranche)
            l.append([ids[i] for i in range(e_from, e_to)])
        return l

    #
    # Dependencies
    #
    def set_dep(self, x, job):
        if x in self.deps:
            raise Exception('Job %s already defined' % str(x))
        self.deps[x] = job

    def get_dep(self, x):
        if not x in self.deps:
            raise Exception('No job %s defined' % str(x))
        return self.deps[x]

    def set_dep_agent_has_learned(self, id_robot, id_agent, job):
        self.set_dep((id_robot, id_agent, 'learn'), job)

    def dep_agent_has_learned(self, id_robot, id_agent):
        """ The agent has completed learning """
        return self.get_dep((id_robot, id_agent, 'learn'))

    def dep_episode_done(self, id_robot, id_episode):
        """ The given episode was completed """
        return self.get_dep((id_robot, id_episode, 'episode'))

    def set_dep_episode_done(self, id_robot, id_episode, job):
        self.set_dep((id_robot, id_episode, 'episode'), job)

    @contract(servo='None|dict',
              servonav='None|dict',
              predict='None|dict')
    def main(self, agents, robots,
                    explore=None,
                    servo=None, servonav=None, predict=None):

        if not robots:
            raise SemanticMistake('Please specify at least one robot.')

        if not agents:
            raise SemanticMistake('Please specify at least one agent.')

        if explore is not None:
            for id_robot in robots:
                self.add_tasks_explore(id_robot=id_robot, **explore)

        for id_robot, id_agent in  itertools.product(robots, agents):
            compatible = are_compatible(data_central=self.data_central,
                                        id_robot=id_robot, id_agent=id_agent)
            if not compatible:
                logger.info('Avoiding combination %s / %s' %
                             (id_robot, id_agent))
                continue

            # FIXME: num_ep_expl (should work also for logs)
            num_ep_expl = explore['num_episodes']
            self.add_learning(id_robot, id_agent, num_ep_expl,
                              explorer=explore['explorer'], # XXX
                              publish_progress=False)

            if servo is not None:
                self.add_tasks_servo(id_agent=id_agent, id_robot=id_robot,
                                     **servo)

            if servonav is not None:
                self.add_tasks_servonav(id_agent=id_agent, id_robot=id_robot,
                                        **servonav)

            if predict is None:
                predict = {}
            self.add_tasks_predict(id_agent=id_agent, id_robot=id_robot,
                                       **predict)

    def add_tasks_predict(self, id_agent, id_robot):
        has_predictor = self.agent_has_predictor(id_agent)
        if not has_predictor:
            #logger.debug('Agent %s does not support predicting.'
            #             % id_agent)
            return

        extra_dep = self.dep_agent_has_learned(id_robot=id_robot,
                                               id_agent=id_agent)

        # FIXME: here we are using *all* streams 
        comp(task_predict, data_central=self.data_central,
             id_agent=id_agent, id_robot=id_robot,
             interval_print=5,
             job_id='predict-%s-%s' % (id_robot, id_agent),
             extra_dep=extra_dep)

    def add_learning(self, id_robot, id_agent, num_ep_expl, explorer, # XXX
                     publish_progress=False):
        all_id_episodes = [self.episode_id_exploration(explorer, i)
                           for i in range(num_ep_expl)]

        def get_deps_for_episodes(id_episodes):
            """ Gets all dependencies for the episodes. """
            return list(set(self.dep_episode_done(id_robot=id_robot,
                                                  id_episode=x)
                            for x in id_episodes))

        previous_state = None
        tranches = self.get_tranches(all_id_episodes)
        for t, id_episodes in enumerate(tranches):
            reset = (t == 0)

            extra_dep = get_deps_for_episodes(id_episodes)
            if previous_state is not None:
                extra_dep.append(previous_state)

            previous_state = comp(learn_log, data_central=self.data_central,
                                  id_agent=id_agent,
                                  id_robot=id_robot,
                                  reset=reset,
                                  episodes=id_episodes,
                                  publish_interval=None,
                                  publish_once=False,
                                  interval_save=300,
                                  interval_print=30,
                                  extra_dep=extra_dep,
                                  job_id='learn-%s-%s-%sof%s' %
                                    (id_robot, id_agent, t + 1, len(tranches)))

            if publish_progress:
                comp(publish_once, self.data_central, id_agent, id_robot,
                     phase='learn', progress='t%03d' % t,
                     job_id='report-learn-%s-%s-%s' % (id_robot, id_agent, t),
                     extra_dep=previous_state)

        all_learned = comp(checkpoint, 'all learned',
                            job_id='learn-%s-%s' % (id_robot, id_agent),
                            extra_dep=[previous_state])

        self.set_dep_agent_has_learned(id_robot=id_robot, id_agent=id_agent,
                                       job=all_learned)

        comp(publish_once, self.data_central, id_agent, id_robot,
             phase='learn', progress='all',
             job_id='report-learn-%s-%s' % (id_robot, id_agent),
             extra_dep=all_learned)

    def add_tasks_explore(self, id_robot, explorer,
                                num_episodes,
                                episodes_per_tranche=10,
                                   num_episodes_videos=1,
                                   videos=default_expl_videos,
                                   max_episode_len=10):

        if num_episodes_videos > num_episodes:
            msg = ('Requested %d videos for only %d episodes' %
                   (num_episodes_videos, num_episodes))
            raise SemanticMistake(msg)

        # Divide the simulation in parallel tranches
        all_id_episodes = [self.episode_id_exploration(explorer, i)
                           for i in range(num_episodes)]

        # These are episodes for which we want to save extra information
        id_episodes_with_extra = [self.episode_id_exploration(explorer, i)
                                  for i in range(num_episodes_videos)]

        tranches = []

        episodes_tranches = self.get_tranches(all_id_episodes,
                                              episodes_per_tranche)
        for t, id_episodes in enumerate(episodes_tranches):
            write_extra = len(set(id_episodes) &
                              set(id_episodes_with_extra)) > 0

            tranche = comp(simulate,
                            data_central=self.data_central,
                             id_agent=explorer,
                             id_robot=id_robot,
                             max_episode_len=max_episode_len,
                             stateful=False,
                             interval_print=5,
                             num_episodes=len(id_episodes),
                             id_episodes=id_episodes,
                             cumulative=False,
                             write_extra=write_extra,
                            job_id='explore-%s-%s-%sof%s' %
                                    (id_robot, explorer, t + 1,
                                     len(episodes_tranches)))

            tranches.append(tranche)

            for id_episode in id_episodes:
                self.set_dep_episode_done(id_robot=id_robot,
                                          id_episode=id_episode, job=tranche)

        comp(checkpoint, 'all simulations',
                         job_id='simulate-%s' % (id_robot),
                         extra_dep=tranches)

        self.add_videos(id_agent=explorer, id_robot=id_robot,
                        id_episodes=id_episodes_with_extra, videos=videos)

    def add_videos(self, id_agent, id_robot, id_episodes, videos):
        # todo: temporary videos
        for id_episode, id_video in itertools.product(id_episodes, videos):
            config = self.data_central.get_bo_config()
            code_spec = config.specs['videos'][id_video]
            model = code_spec['code'][0]
            model_params = code_spec['code'][1]

            extra_dep = self.dep_episode_done(id_robot, id_episode)

            comp(create_video,
                 data_central=self.data_central,
                 id_episode=id_episode,
                 id_agent=id_agent,
                 id_robot=id_robot,
                 suffix=id_video,
                 model=model,
                 model_params=model_params,
                 job_id='video-%s-%s-%s-%s' %
                    (id_robot, id_agent, id_episode, id_video),
                 extra_dep=extra_dep)

    def add_tasks_servonav(self, id_agent, id_robot,
                                 num_episodes,
                                 episodes_per_tranche=1,
                                 num_episodes_videos=0,
                                 max_episode_len=10,
                                 resolution=1,
                                 fail_if_not_working=False,
                                 videos=default_servonav_videos):

        logger.info('Adding servonav for %s/%s %s %s' %
                    (id_agent, id_robot, num_episodes,
                     num_episodes_videos))

        has_servo = self.agent_has_servo(id_agent)
        if not has_servo:
            logger.debug('Agent %s does not support servoing.' % id_agent)
            return

        if num_episodes_videos > num_episodes:
            raise SemanticMistake('More videos than episodes requested.')

        if num_episodes == 0:
            logger.debug('No servonav episodes')
            return

        all_id_episodes = [self.episode_id_servonav(id_agent, i)
                           for i in range(num_episodes)]
        id_episodes_with_extra = [self.episode_id_servonav(id_agent, i)
                           for i in range(num_episodes_videos)]

        all_tranches = []
        episodes_tranches = self.get_tranches(all_id_episodes,
                                              episodes_per_tranche)
        for st, id_episodes in enumerate(episodes_tranches):
            num_episodes_with_robot_state = len(set(id_episodes) &
                                                set(id_episodes_with_extra))

            tranche = comp(task_servonav, data_central=self.data_central,
             id_agent=id_agent, id_robot=id_robot,
             max_episode_len=max_episode_len,
             num_episodes=len(id_episodes),
             id_episodes=id_episodes,
             cumulative=False,
             resolution=resolution,
             interval_print=5,
             fail_if_not_working=fail_if_not_working,
             num_episodes_with_robot_state=num_episodes_with_robot_state,
             job_id='servonav-%s-%s-%sof%s' %
             (id_robot, id_agent, st + 1, len(episodes_tranches)),
             extra_dep=self.dep_agent_has_learned(id_robot, id_agent))

            all_tranches.append(tranche)

            for id_episode in id_episodes:
                self.set_dep_episode_done(id_robot, id_episode, tranche)

#        logger.info('Tranches: %s' % all_tranches)

        comp(checkpoint, 'all servonav',
                        job_id='servonav-%s-%s' % (id_robot, id_agent),
                        extra_dep=all_tranches)

#        summaries = comp(servo_stats_summaries, self.data_central,
#                         id_agent, id_robot,
#                         job_id=('servo-%s-%s-summary' %
#                                  (id_robot, id_agent)),
#                         extra_dep=all_servonav)
#
#        comp(servo_stats_report, self.data_central, id_agent,
#             id_robot, summaries,
#             job_id='servo-%s-%s-report' % (id_robot, id_agent))

        self.add_videos(id_agent=id_agent,
                          id_robot=id_robot,
                          id_episodes=id_episodes_with_extra,
                          videos=videos)

    def add_tasks_servo(self, id_agent, id_robot,
                        num_episodes,
                        num_episodes_videos=0,
                        max_episode_len=10,
                        displacement=3,
                        videos=default_servo_videos):

        logger.info('Adding servo for %s/%s; %s episodes of which '
                    '%s with video.' %
                    (id_agent, id_robot, num_episodes,
                     num_episodes_videos))

        has_servo = self.agent_has_servo(id_agent)

        if num_episodes_videos > num_episodes:
            raise SemanticMistake('More videos than episodes requested.')

        if not has_servo:
            logger.debug('Agent %s does not support servoing.' % id_agent)
            return

        if num_episodes == 0:
            logger.debug('No servo episodes')
            return

        logger.debug('Creating servo episodes')

        all_id_episodes = [self.episode_id_servoing(id_agent, i)
                           for i in range(num_episodes)]
        id_episodes_with_extra = [self.episode_id_servoing(id_agent, i)
                           for i in range(num_episodes_videos)]

        all_tranches = []
        episodes_tranches = self.get_tranches(all_id_episodes)
        for st, id_episodes in enumerate(episodes_tranches):
            num_episodes_with_robot_state = len(set(id_episodes) &
                                                set(id_episodes_with_extra))

            tranche = comp(task_servo, data_central=self.data_central,
             id_agent=id_agent, id_robot=id_robot,
             max_episode_len=max_episode_len,
             num_episodes=len(id_episodes),
             id_episodes=id_episodes,
             cumulative=False,
             displacement=displacement,
             interval_print=5,
             num_episodes_with_robot_state=num_episodes_with_robot_state,
             job_id='servo-%s-%s-%sof%s' % (id_robot, id_agent, st + 1,
                                             len(episodes_tranches)),
             extra_dep=self.dep_agent_has_learned(id_robot, id_agent))

            all_tranches.append(tranche)

            for id_episode in id_episodes:
                self.set_dep_episode_done(id_robot, id_episode, tranche)

        all_servo = comp(checkpoint, 'all servo',
                        job_id='servo-%s-%s' % (id_robot, id_agent),
                        extra_dep=all_tranches)

        summaries = comp(servo_stats_summaries, self.data_central,
                         id_agent=id_agent, id_robot=id_robot,
                         id_episodes=all_id_episodes,
                         job_id=('servo-%s-%s-summary' %
                                  (id_robot, id_agent)),
                         extra_dep=all_servo)

        comp(servo_stats_report, self.data_central, id_agent,
             id_robot, summaries,
             job_id='report-servo-%s-%s' % (id_robot, id_agent))

        self.add_videos(id_agent=id_agent,
                          id_robot=id_robot,
                          id_episodes=id_episodes_with_extra,
                          videos=videos)


def are_compatible(data_central, id_robot, id_agent):
    # XXX: this is wasteful
    config = data_central.get_bo_config()
    robot = config.robots.instance(id_robot)
    agent = config.agents.instance(id_agent)
    try:
        agent.init(robot.get_spec())
    except UnsupportedSpec as e:
        logger.debug('%s/%s: %s' % (id_robot, id_agent, e))
        return False
    return True


def checkpoint(msg):
    ''' Dummy function for job. '''
    #print(msg)
