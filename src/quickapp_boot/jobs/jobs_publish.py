from quickapp_boot.programs import PublishLearningResult
from quickapp_boot.utils import iterate_context_agents
from quickapp.library.context.compmake_context import CompmakeContext
from contracts import contract

    
@contract(context=CompmakeContext, boot_root='str', agents='list(str)', id_robot='str')
def jobs_publish_learning_agents(context, boot_root, agents, id_robot):
    for c, id_agent in iterate_context_agents(context, agents):
        jobs_publish_learning(c, boot_root, id_agent, id_robot)
        
@contract(context=CompmakeContext, boot_root='str', id_agent='str', id_robot='str')    
def jobs_publish_learning(context, boot_root, id_agent, id_robot):
    context.needs('agent-learn', id_agent=id_agent, id_robot=id_robot)
    context.subtask(PublishLearningResult,
                    boot_root=boot_root, agent=id_agent, robot=id_robot)
