
from quickapp import QuickApp

__all__ = ['LearnLogNoSaveHint', 'LearnLogNoSaveHintRepeated']


class LearnLogNoSaveHint(QuickApp):
    cmd = 'learn-log-nosave-hint'
     
    '''
        Runs the learning for a given agent and set of episodes,
        but does not save the state, ever, but the agent
        itself is returned for use.
        
        Here, the agent is also given the parallel hint.          
    '''
     
    def define_options(self, params):
        params.add_string("boot_root", help="Root boot data")
        params.add_string("agent", help="Agent ID")
        params.add_string("robot", help="Robot ID")
        
        params.add_int("interval_print", default=5, help="Interval for "
                       "printing stats (seconds)")
        
        params.add_string_list("episodes", help="List of episodes to learn, "
                               "or None to mean all episodes.")
 
        params.add_int_list("parallel_hint", help="Parallel hint")
        
 
    def define_jobs_context(self, context):
        options = self.get_options()
        data_central = DataCentral(options.boot_root)
         
        return context.comp(learn_log,
                            reset=True,
                            data_central=data_central,
                              id_agent=options.agent,
                              id_robot=options.robot,
                              episodes=options.episodes,
                              publish_interval=None,
                              publish_once=False,
                              save_state=False,
                              parallel_hint=tuple(options.parallel_hint))



class LearnLogNoSaveHintRepeated(QuickApp):
    cmd = 'learn-log-nosave-hint-repeated'
     
    '''
        Runs the learning for a given agent and set of episodes,
        but does not save the state, ever, but the agent
        itself is returned for use.
        
        Also run for 
        
        Here, the agent is also given the parallel hint.          
    '''
     
    def define_options(self, params):
        params.add_string("boot_root", help="Root boot data")
        params.add_string("agent", help="Agent ID")
        params.add_string("robot", help="Robot ID")
        params.add_int("max_reps", help="Maximum number of repetitions")
        
        params.add_int("interval_print", default=5, help="Interval for "
                       "printing stats (seconds)")
        
        params.add_string_list("episodes", help="List of episodes to learn, "
                               "or None to mean all episodes.")
 
        params.add_int_list("parallel_hint", help="Parallel hint [i, n]")
        params.add_flag('intermediate_reports', help='Produces a report after each repetition.')
 
    def define_jobs_context(self, context):
        options = self.get_options()
        data_central = DataCentral(options.boot_root)
        id_agent = options.agent
        id_robot = options.robot
        episodes = options.episodes
        parallel_hint = tuple(options.parallel_hint)
        
        # First time, creating the state
        first_ast = context.comp_config(learn_log,
                                    reset=True,
                                    data_central=data_central,
                                  id_agent=id_agent,
                                  id_robot=id_robot,
                                  episodes=episodes,
                                  publish_interval=None,
                                  publish_once=False,
                                  save_state=False,
                                  parallel_hint=options.parallel_hint,
                                  job_id='learn-rep0')
        
        ast = first_ast
        for i in range(self.options.max_reps - 1):
            
            if self.options.intermediate_reports:
                progress = 'rep%d' % i
                report = context.comp(get_agentstate_report, ast, progress,
                                      job_id='learn-rep%d-report' % i)
                context.add_report(report, 'agent_report_partial_rep',
                                   id_agent=id_agent, id_robot=id_robot,
                                   progress=progress, parallel_hint=parallel_hint[0])

            # Other times calling "learn_log_base"
            ast = context.comp_config(learn_log_base,
                                      data_central=data_central,
                                      id_agent=id_agent,
                                      agent_state=ast, id_robot=id_robot,
                                      episodes=episodes,
                                      ignore_learned=True,
                                      job_id='learn-rep%d' % (i + 1))
            
        return ast 



