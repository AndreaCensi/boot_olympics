
from boot_manager import DataCentral
from boot_manager.meat.publish_output import get_agent_report
from quickapp import QuickApp

__all__ = ['PublishLearningResult', 'jobs_publish_learn_report']


# Deprecated
class PublishLearningResult(QuickApp):
    cmd = 'learn-publish'
    usage = 'learn-publish --agent <AGENT> --robot <ROBOT>  '
     
    '''
        Publishes the learning result.
    '''
     
    def define_options(self, params):
        params.add_string("boot_root", help="Root boot data")
        params.add_string("agent", help="Agent ID")
        params.add_string("robot", help="Robot ID")
        params.add_string("progress", help="name of current phase", default='all')
        
    def define_jobs_context(self, context):
        options = self.get_options()
        data_central = DataCentral(options.boot_root)
        jobs_publish_learn_report(context ,data_central,  
                                  id_agent=options.id_agent, 
                                  id_robot=options.id_robot, progress='all')

        
def jobs_publish_learn_report(context, data_central, id_agent,
                              id_robot, progress='all'):
        key = dict(id_agent=id_agent,
                  id_robot=id_robot,
                  progress=progress)
        report = context.comp_config(get_agent_report,
                              data_central=data_central,
                              **key)
        context.add_report(report, 'agent_report_partial', **key)
    
