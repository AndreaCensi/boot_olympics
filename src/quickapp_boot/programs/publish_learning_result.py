from bootstrapping_olympics.programs.manager import DataCentral, publish_once
from quickapp import QuickApp

__all__ = ['PublishLearningResult']


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
        
    def define_jobs_context(self, context):
        options = self.get_options()
        data_central = DataCentral(options.boot_root)
         
        context.comp(publish_once,
                     data_central=data_central,
                     id_agent=options.agent,
                     id_robot=options.robot,
                     phase='learn', progress='all')
