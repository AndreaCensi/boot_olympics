from quickapp.compmake_context import CompmakeContext
from contracts import contract
from quickapp_boot import REP_SERVO_AGENT, RM_AGENT_LEARN
from reprep import Report
from bootstrapping_olympics import ServoAgentInterface

@contract(context=CompmakeContext, id_agent='str', id_robot='str', create_report='bool')
def jobs_servo_agent(context, id_agent, id_robot, create_report=True):
    """ Creates a report for the servo_agent. Returns the servo agent """
    agent_state = context.get_resource(RM_AGENT_LEARN, id_agent=id_agent, id_robot=id_robot)
    servo_agent = context.comp(get_servo_agent, agent_state)
    if create_report:
        report = context.comp(get_servo_agent_report, servo_agent)
        context.add_report(report, REP_SERVO_AGENT, id_agent=id_agent,
                           id_robot=id_robot)
    return servo_agent

@contract(agent_state='tuple(*,*)', returns=ServoAgentInterface)
def get_servo_agent(agent_state):
    agent, _ = agent_state
    return agent.get_servo()

@contract(servo_agent=ServoAgentInterface)
def get_servo_agent_report(servo_agent):
    r = Report()
    servo_agent.publish(r)
    return r


    