from boot_manager import DataCentral
from quickapp import QuickApp

__all__ = ['LearnLogNoSave']


class LearnLogNoSave(QuickApp):
    cmd = 'learn-log-nosave'
     
    '''
        Runs the learning for a given agent and set of episodes,
        but does not save the state, ever, but the agent
        itself is returned for use.          
    '''
     
    def define_options(self, params):
        params.add_string("boot_root", help="Root boot data")
        params.add_string("agent", help="Agent ID")
        params.add_string("robot", help="Robot ID")
        
        params.add_int("interval_print", default=5, help="Interval for "
                       "printing stats (seconds)")
        
        params.add_string_list("episodes", help="List of episodes to learn, "
                               "or None to mean all episodes.")
 
 
    def define_jobs_context(self, context):
        options = self.get_options()
        data_central = DataCentral(options.boot_root)
         
        return context.comp_config(learn_log,
                                   reset=True,
                                   data_central=data_central,
                                  id_agent=options.agent,
                                  id_robot=options.robot,
                                  episodes=options.episodes,
                                  publish_interval=None,
                                  publish_once=False,
                                  save_state=False)

