'''Some functions to help in writing experiments scripts'''
from . import default_expl_videos, default_servo_videos, default_servonav_videos
from .. import (create_video, servo_stats_report, simulate, task_predict,
    learn_log, publish_once, task_servo, task_servonav, publish_report_robot,
    servo_stats_summary, predict_report)
from bootstrapping_olympics import UnsupportedSpec, logger
from bootstrapping_olympics.library.robots import EquivRobot
from bootstrapping_olympics.programs.manager.meat.nuislog import (
    nuislog_episodes)
from conf_tools import SemanticMistake
from contracts import contract
import itertools
import numpy as np

def batch_jobs1(data_central, **kwargs):
    tr = TaskRegister(data_central)
    tr.main(**kwargs)


class TaskRegister:

    def __init__(self, data_central):
        self.data_central = data_central

        self.deps = {}
        # Instances we keep around to probe
        self.agent_instances = {}
        self.robot_instances = {}
        
    def _get_agent_instance(self, id_agent):
        if not id_agent in self.agent_instances:
            self.agent_instances[id_agent] = \
                 self.data_central.get_bo_config().agents.instance(id_agent)
        return  self.agent_instances[id_agent] 

    def _get_robot_instance(self, id_robot):
        if not id_robot in self.robot_instances:
            self.robot_instances[id_robot] = \
                 self.data_central.get_bo_config().robots.instance(id_robot)
        return  self.robot_instances[id_robot] 

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
        agent = self._get_agent_instance(id_agent)
        return hasattr(agent, 'get_predictor')

    def agent_has_servo(self, id_agent):
        agent = self._get_agent_instance(id_agent)
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

    @contract(returns='list(str)')
    def get_all_exploration_episodes(self, id_robot):
        pass

    @contract(servo='None|dict',
              servonav='None|dict',
              predict='None|dict')
    def main(self, agents, robots,
                    explore=None,
                    explore_modulus=None,
                    servo=None,
                    servonav=None,
                    predict=None):

        if predict is None:
            logger.info('No prediction specified.')
            
        if not robots:
            raise SemanticMistake('Please specify at least one robot.')

        if not agents:
            raise SemanticMistake('Please specify at least one agent.')

        for id_robot in robots:
            self.add_task_robot_report(id_robot=id_robot)
            
        if explore_modulus is not None:
            self.add_tasks_explore_modulus(robots, **explore_modulus)
            num_ep_expl = explore_modulus['num_episodes']
            explorer = explore_modulus['explorer']
        # TODO: validate inputs
        if explore is not None:
            for id_robot in robots:
                self.add_tasks_explore(id_robot=id_robot, **explore)
            num_ep_expl = explore['num_episodes']
            explorer = explore['explorer']
            
        for id_robot, id_agent in itertools.product(robots, agents):
            compatible, reason = self.are_compatible(id_robot, id_agent)
            if not compatible:
                logger.info('Avoiding combination %s / %s: %s' % 
                             (id_robot, id_agent, reason))
                continue

            # FIXME: num_ep_expl (should work also for logs)
            self.add_learning(id_robot, id_agent, num_ep_expl=num_ep_expl,
                              explorer=explorer,  # XXX
                              publish_progress=False,
                              save_pickle=True)  # TODO: make param

            if servo is not None:
                self.add_tasks_servo(id_agent=id_agent, id_robot=id_robot,
                                     **servo)

            if servonav is not None:
                self.add_tasks_servonav(id_agent=id_agent, id_robot=id_robot,
                                        **servonav)

            if predict is not None:
                self.add_tasks_predict(id_agent=id_agent, id_robot=id_robot,
                                       **predict)

    def compmake_job(self, *args, **kwargs):
        """ Calls compmake's self.compmake_job() function. """    
        try:
            from compmake import comp
        except ImportError:
            logger.error('Compmake not installed')
        return comp(*args, **kwargs)
    
    def add_task_robot_report(self, id_robot):
        self.compmake_job(publish_report_robot, data_central=self.data_central,
              id_robot=id_robot, save_pickle=True,
             job_id='report-robot-%s' % (id_robot))
        
    def add_tasks_predict(self, id_agent, id_robot, live_plugins=[],
                          save_pickle=True):
        if not self.agent_has_predictor(id_agent):
            return

        extra_dep = self.dep_agent_has_learned(id_robot=id_robot,
                                               id_agent=id_agent)

        # FIXME: here we are using *all* streams 
        statistics = self.compmake_job(task_predict, data_central=self.data_central,
             id_agent=id_agent, id_robot=id_robot, live_plugins=live_plugins,
             job_id='predict-%s-%s' % (id_robot, id_agent),
             extra_dep=extra_dep)
        
        self.compmake_job(predict_report, data_central=self.data_central,
                             id_agent=id_agent, id_robot=id_robot,
                             statistics=statistics, save_pickle=save_pickle,
             job_id='report-predict-%s-%s' % (id_robot, id_agent))

    def add_learning(self, id_robot, id_agent, num_ep_expl, explorer,  # XXX
                     publish_progress=False, save_pickle=False,
                     episodes_per_tranche=500):
        all_id_episodes = [self.episode_id_exploration(explorer, i)
                           for i in range(num_ep_expl)]

        def get_deps_for_episodes(id_episodes):
            """ Gets all dependencies for the episodes. """
            return list(set(self.dep_episode_done(id_robot=id_robot,
                                                  id_episode=x)
                            for x in id_episodes))

        previous_state = None
        tranches = self.get_tranches(all_id_episodes, episodes_per_tranche)
        for t, id_episodes in enumerate(tranches):
            reset = (t == 0)

            extra_dep = get_deps_for_episodes(id_episodes)
            if previous_state is not None:
                extra_dep.append(previous_state)

            job_id = ('learn-%s-%s-%sof%s' % 
                      (id_robot, id_agent, t + 1, len(tranches)))
            previous_state = self.compmake_job(learn_log,
                                               data_central=self.data_central,
                                              id_agent=id_agent,
                                              id_robot=id_robot,
                                              reset=reset,
                                              episodes=id_episodes,
                                              publish_interval=None,
                                              publish_once=False,
                                              interval_save=300,
                                              interval_print=30,
                                              extra_dep=extra_dep,
                                              job_id=job_id)

            if publish_progress:
                self.compmake_job(publish_once, self.data_central, id_agent, id_robot,
                     phase='learn', progress='t%03d' % t,
                     job_id='report-learn-%s-%s-%s' % (id_robot, id_agent, t),
                     extra_dep=previous_state)

        all_learned = self.compmake_job(checkpoint, 'all learned',
                            job_id='learn-%s-%s' % (id_robot, id_agent),
                            extra_dep=[previous_state])

        self.set_dep_agent_has_learned(id_robot=id_robot, id_agent=id_agent,
                                       job=all_learned)

        self.compmake_job(publish_once, self.data_central, id_agent, id_robot,
             phase='learn', progress='all', save_pickle=save_pickle,
             job_id='report-learn-%s-%s' % (id_robot, id_agent),
             extra_dep=all_learned)

    def add_tasks_explore_modulus(self, robots, episodes_per_tranche=50, **explore_args):
        """ 
            Adds exploration tasks, making sure to use the same 
            data for robots up to nuisances. 
        """
        # ID robot -> (obsn, original, cmdn)
        robot2canonical = {}
        
        # (original, cmd) -> list(str)
        from collections import defaultdict
        core2robots = defaultdict(list)
        
        for id_robot in robots:
            canform = self.get_robot_modulus(id_robot)
            robot2canonical[id_robot] = canform
            _, original, cmdn = canform
            core2robots[(original, cmdn)].append(id_robot)
            
            logger.info('Canonical form of %r: %s' % 
                        (id_robot, robot2canonical[id_robot]))
            
        config = self.data_central.get_bo_config()
        
        for original, cmd in core2robots:
            derived = core2robots[(original, cmd)]
            logger.info('Core robot (%s,%s) corresponds to %s' % 
                        (original, cmd, derived))
            
            # XXX: not sure of order
            new_robot_name = "".join(['U%s' % x for x in cmd]) + original
            if not new_robot_name in config.robots:
                msg = 'Bug found, or bad naming practices.'
                msg += 'I want to instantiate a %r but not found.' % new_robot_name
                raise Exception(msg)
            
            explore_args['episodes_per_tranche'] = episodes_per_tranche
            id_episodes = self.add_tasks_explore(new_robot_name, **explore_args)
            logger.info('Defined %d episodes for %r.' % (len(id_episodes), new_robot_name))
            
            # Now convert one episode into the other
            for id_derived in derived:
                logger.info('Considering derived %s ' % id_derived)
                derived_obs, x, derived_cmd = robot2canonical[id_derived]
                assert x == original
                assert derived_cmd == cmd
                if not derived_obs:
                    logger.info(' ... skipping because pure.')
                    # this is a pure robot
                    continue
                
                episodes_tranches = self.get_tranches(id_episodes,
                                                      episodes_per_tranche)
                
                for i, tranche in enumerate(episodes_tranches):
                    job_id = 'derive-%s-%d' % (id_derived, i)
                    
                    extra_dep = []
                    for id_episode in tranche:
                        extra_dep.append(self.dep_episode_done(new_robot_name,
                                                               id_episode))
                        
                    job = self.compmake_job(nuislog_episodes,
                                            data_central=self.data_central,
                                            id_robot_original=new_robot_name,
                                            id_episodes=tranche,
                                            id_equiv=id_derived,
                                            obs_nuisances=derived_obs,
                                            cmd_nuisances=[],  # <- correct
                                            with_extras=True,
                                            job_id=job_id,
                                            extra_dep=extra_dep)
                    
                    for id_episode in tranche:
                        self.set_dep_episode_done(id_robot=id_derived,
                                                  id_episode=id_episode, job=job)
     
    
    @contract(returns='tuple(tuple, str, tuple)')
    def get_robot_modulus(self, id_robot):
        """ Returns the canonical description of a robot,
            as a list of nuisances on observations,
            robot name,
            nuisances on commands. 
        
            obsn, original, cmdn = self.get_robot_modulus() 
        """
        
        robot = self._get_robot_instance(id_robot)
        if isinstance(robot, EquivRobot):
            return robot.get_robot_modulus()
        else:
            return (tuple([]), id_robot, tuple([]))


    @contract(returns='list(str)')
    def add_tasks_explore(self, id_robot, explorer,
                                num_episodes,
                                episodes_per_tranche=10,
                                num_episodes_videos=1,
                                videos=default_expl_videos,
                                max_episode_len=10):
        """ Returns the episodes id """

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

            tranche = self.compmake_job(simulate,
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

        self.compmake_job(checkpoint, 'all simulations',
                         job_id='simulate-%s' % (id_robot),
                         extra_dep=tranches)

        self.add_videos(id_agent=explorer, id_robot=id_robot,
                        id_episodes=id_episodes_with_extra, videos=videos)

        return all_id_episodes

    def add_videos(self, id_agent, id_robot, id_episodes, videos):
        # todo: temporary videos
        for id_episode, id_video in itertools.product(id_episodes, videos):
            config = self.data_central.get_bo_config()
            code_spec = config.specs['videos'][id_video]
            model = code_spec['code'][0]
            model_params = code_spec['code'][1]

            extra_dep = self.dep_episode_done(id_robot, id_episode)

            self.compmake_job(create_video,
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
 
        has_servo = self.agent_has_servo(id_agent)
        if not has_servo:
            return

        if num_episodes_videos > num_episodes:
            raise SemanticMistake('More videos than episodes requested.')

        if num_episodes == 0:
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

            tranche = self.compmake_job(task_servonav, data_central=self.data_central,
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

        self.compmake_job(checkpoint, 'all servonav',
                        job_id='servonav-%s-%s' % (id_robot, id_agent),
                        extra_dep=all_tranches)
 
        self.add_videos(id_agent=id_agent,
                          id_robot=id_robot,
                          id_episodes=id_episodes_with_extra,
                          videos=videos)

    def add_tasks_servo(self, id_agent, id_robot,
                        num_episodes,
                        num_episodes_videos=0,
                        max_episode_len=10,
                        displacement=3,
                        episodes_per_tranche=1,
                        videos=default_servo_videos):
 
        if num_episodes_videos > num_episodes:
            raise SemanticMistake('More videos than episodes requested.')

        if not self.agent_has_servo(id_agent):
            return

        if num_episodes == 0:
            return 

        all_id_episodes = [self.episode_id_servoing(id_agent, i)
                           for i in range(num_episodes)]
        id_episodes_with_extra = [self.episode_id_servoing(id_agent, i)
                           for i in range(num_episodes_videos)]
        summaries = []
        all_tranches = []
        episodes_tranches = self.get_tranches(all_id_episodes, episodes_per_tranche)
        for st, id_episodes in enumerate(episodes_tranches):
            num_episodes_with_robot_state = len(set(id_episodes) & 
                                                set(id_episodes_with_extra))

            tranche = self.compmake_job(task_servo,
                                        data_central=self.data_central,
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
                
            for id_episode in id_episodes:
                job_id = ('servo-%s-%s-%s-summary' % 
                          (id_robot, id_agent, id_episode))
                summary = self.compmake_job(servo_stats_summary,
                                            self.data_central, id_agent,
                                            id_robot, id_episode,
                                            job_id=job_id,
                                            extra_dep=[tranche])
                summaries.append(summary)

        self.compmake_job(checkpoint, 'all servo',
                        job_id='servo-%s-%s' % (id_robot, id_agent),
                        extra_dep=all_tranches) 
         
        self.compmake_job(servo_stats_report, self.data_central, id_agent,
             id_robot, summaries,
             job_id='report-servo-%s-%s' % (id_robot, id_agent))

        # TODO: create partial summaries
        ntot = len(summaries)
        num = ntot
        for i in range(ntot):
            num = num / 2
            if num <= 2:
                continue
            psummaries = summaries[:num]
            # logger.info('Creatint partial summaries with %d/%d using %s' % (num, ntot, psummaries))
            self.compmake_job(servo_stats_report, self.data_central, id_agent,
                               id_robot, psummaries,
                               phase='servo_stats-partial%d' % i,
                job_id='report-servo-%s-%s-partial%d' % (id_robot, id_agent, i))
   
        self.add_videos(id_agent=id_agent,
                          id_robot=id_robot,
                          id_episodes=id_episodes_with_extra,
                          videos=videos)

    @contract(returns='tuple(bool, None|str)')
    def are_compatible(self, id_robot, id_agent):
        agent = self._get_agent_instance(id_agent)
        robot = self._get_robot_instance(id_robot)
        
        try:
            agent.init(robot.get_spec())
        except UnsupportedSpec as e:
            # logger.debug('%s/%s: %s' % (id_robot, id_agent, e))
            return False, str(e)
        
        return True, None


def checkpoint(msg):
    ''' Dummy function for job. '''
    # print(msg)
