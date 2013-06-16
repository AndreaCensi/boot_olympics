from bootstrapping_olympics.programs.manager import DataCentral
from bootstrapping_olympics.programs.manager.meat.publish_output import (
    get_agent_report)
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
        params.add_string("progress", help="name of current phase", default='all')
        
    def define_jobs_context(self, context):
        options = self.get_options()
        data_central = DataCentral(options.boot_root)
         
        key = dict(id_agent=options.agent,
                  id_robot=options.robot,
                  progress=options.progress)
        report = context.comp_config(get_agent_report,
                              data_central=data_central,
                              **key)
        context.add_report(report, 'agent_report_partial', **key)
