from ..meat import learn_log
from .main import BOM


class CmdLearnLog(BOM.get_sub()):
    '''
        Runs the learning for a given agent and log. 
       
       
        Running a live plugin:
        
            bom  [agent/robot options] --reset --dontsave --plugin dummy
        
    '''
    cmd = 'learn-log'

    def define_program_options(self, params):

        params.add_string('agent', short="-a",  help="Agent ID")
        params.add_string('robot', short="-r",  help="Robot ID")
        params.add_flag("reset", help="Do not use cached state.")
        params.add_float("publish", short="-p",   
                          default=None,
                          help="Publish debug information every N cycles.")
        params.add_flag("once",
                          help="Just plot the published information and exit.")
#         params.add_int("interval_save",   default=300,
#                           help="Interval for saving state (seconds) [%default]")
#         params.add_int("interval_print",  default=5,
#                           help="Interval for printing stats (seconds) [%default]")
        params.add_flag("dontsave", 
                          help="Do not save the state of the agent.")

        params.add_string_list("plugin", default=[],
                          help="Run the specified plugin model during "
                               "learning. (eg, visualization)")


    def go(self):
        options = self.get_options()
        data_central = self.get_parent().get_data_central()

        if options.publish is  None and not options.once:
            msg = 'Not creating any report; pass -p <interval> or --once to do it.'
            self.info(msg)
            
        self.error('Implement publish()')
              
        learn_log(data_central=data_central,
                  id_agent=options.agent,
                  id_robot=options.robot,
                  reset=options.reset,
#                   publish_interval=options.publish,
#                   publish_once=options.once,
#                   interval_save=options.interval_save,
#                   interval_print=options.interval_print,
                  save_state=not(options.dontsave),
#                   live_plugins=options.plugin,
                  )
