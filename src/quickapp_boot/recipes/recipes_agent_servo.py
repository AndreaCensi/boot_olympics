from contracts import contract
from quickapp import CompmakeContext
from quickapp_boot import RM_AGENT_SERVO
from quickapp_boot.jobs.jobs_servo import jobs_servo_agent

__all__ = ['recipe_agent_servo']

@contract(context=CompmakeContext, create_report=bool)
def recipe_agent_servo(context, create_report=True):
    """
        provides:  RM_AGENT_SERVO (id_agent, id_robot)
        
    """
    
    def rp_servo(context, id_agent, id_robot):
        servo = jobs_servo_agent(context, id_agent, id_robot, create_report=create_report)
        return servo
    
    rm = context.get_resource_manager()    
    rm.set_resource_provider(RM_AGENT_SERVO, rp_servo)


