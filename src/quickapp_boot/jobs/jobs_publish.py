from contracts import contract
from quickapp import CompmakeContext
from quickapp_boot.utils import iterate_context_agents, iterate_context_robots
from quickapp_boot import RM_AGENT_LEARN
from bootstrapping_olympics.programs.manager.meat.publish_output import get_agent_report
from bootstrapping_olympics.programs.manager.meat.data_central import DataCentral

    
@contract(context=CompmakeContext, boot_root='str', agents='list(str)', id_robot='str')
def jobs_publish_learning_agents(context, boot_root, agents, id_robot):
    for c, id_agent in iterate_context_agents(context, agents):
        jobs_publish_learning(c, boot_root, id_agent, id_robot)


@contract(context=CompmakeContext, boot_root='str', agents='list(str)', robots='list(str)')
def jobs_publish_learning_agents_robots(context, boot_root, agents, robots):
    for cr, id_robot in iterate_context_robots(context, robots):
        for c, id_agent in iterate_context_agents(cr, agents):
            jobs_publish_learning(c, boot_root, id_agent, id_robot)

        
@contract(context=CompmakeContext, boot_root='str', id_agent='str', id_robot='str')    
def jobs_publish_learning(context, boot_root, id_agent, id_robot):
    learn = context.get_resource(RM_AGENT_LEARN, id_agent=id_agent, id_robot=id_robot)
    
    report = context.comp_config(get_agent_report,
                                 data_central=DataCentral(boot_root),
                                 id_agent=id_agent, id_robot=id_robot, progress='all',
                                 extra_dep=[learn])
                          
    context.add_report(report, 'agent_report_partial',
                       id_agent=id_agent, id_robot=id_robot, progress='all')


# This has data_central instead of boot_root
@contract(context=CompmakeContext, id_agent='str', id_robot='str')
def jobs_publish_learning2(context, data_central, id_agent, id_robot):
    learn = context.get_resource(RM_AGENT_LEARN, id_agent=id_agent, id_robot=id_robot)

    report = context.comp_config(get_agent_report,
                                 data_central=data_central,
                                id_agent=id_agent, id_robot=id_robot, progress='all',
                                extra_dep=[learn])

    context.add_report(report, 'agent_report_partial',
                       id_agent=id_agent, id_robot=id_robot, progress='all')
