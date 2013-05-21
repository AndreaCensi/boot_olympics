try:
    from .. import BootstrappingCommands, BootstrappingObservations, rospy
    from ..publisher import ROSPublisher
    from ..ros_logs import ROS2Python, boot_spec_from_ros_message
    from ..ros_script_utils import RospyLogger
except:  # allow to run nose even if ros is not installed
    pass
from bootstrapping_olympics import AgentInterface
from bootstrapping_olympics.programs.manager.meat import (DataCentral,
    load_agent_state_core)
from bootstrapping_olympics.utils import check_parameters
from contracts import contract
from pprint import pformat


class Global:
    observations = None
    logger = None


class BootAgentAdapter():

    @contract(publish_interval='>=0')
    def __init__(self, data_central, id_agent, agent, publish_interval):
        self.logger = Global.logger
        self.count = 0
        self.publish = publish_interval > 0
        self.agent = agent
        self.id_agent = id_agent
        if self.publish:
            self.ros_publisher = ROSPublisher()

        # Wait until the robot is connected
        self.logger.info('Waiting for robot...')
        rospy.wait_for_service('commands')  # @UndefinedVariable
        self.logger.info('...connected to robot.')
        self.set_commands = rospy.ServiceProxy('commands',
                                               BootstrappingCommands)

        self.logger.info('Waiting for one observation...')
        observations = read_one_observation()
        if observations is None:
            raise Exception('Interrupted')
        id_robot = observations.id_robot
        self.logger.info('...observations received from %r.' % id_robot)
        spec = boot_spec_from_ros_message(observations)

        # agent initialization
        agent.init(spec)
        load_agent_state_core(data_central, id_agent, agent, id_robot,
                              reset_state=False,
                              raise_if_no_state=False)

        self.ros2python = ROS2Python(spec)

        # Subscribe to the observations
        self.subscriber = rospy.Subscriber('observations',  # @UndefinedVariable
                                           BootstrappingObservations,
                                           self.event_loop)

    def go(self):
        rospy.spin()  # @UndefinedVariable

    def event_loop(self, observations):
        obs = self.ros2python.convert(observations, filter_doubles=True)
        if obs is not None:  # possibly repeated
            self.agent.process_observations(obs)
        # else:
            # return # XXXX 

        commands = self.agent.choose_commands()

        if self.publish and self.count % self.publish_interval == 0:
            self.agent.publish(self.ros_publisher)
        self.count += 1

        # TODO: check commands is compatible with commands_spec
        self.set_commands(commands=commands.tolist(),
                          sender=self.id_agent,
                          timestamp=observations.timestamp)


def boot_agent_adapter_main(params):
    Global.logger = RospyLogger('agent_adapter')

    Global.logger.info('My params:\n%s' % pformat(params))

    check_parameters(params, required=['agent_spec', 'root'],
                             optional=['publish_interval'])

    data_central = DataCentral(params['root'])
    bo_config = data_central.get_bo_config()

    publish_interval = params.get('publish_interval', 0)
    agent_spec = params['agent_spec']
    #     check_valid_agent_config(agent_spec)
    agent = bo_config.agents.instance_spec(agent_spec)  # @UndefinedVariable
    id_agent = agent_spec['id']

    AgentInterface.logger = RospyLogger(id_agent)

    agent_adapter = BootAgentAdapter(data_central=data_central,
                                     id_agent=id_agent,
                                     agent=agent,
                                     publish_interval=publish_interval)

    agent_adapter.go()


def read_one_observation(sleep=0.1):
    logger = Global.logger

    def handle_observations(resp):
        Global.observations = resp

    # Subscribe to the observations
    subscriber = rospy.Subscriber('observations', BootstrappingObservations,
                     handle_observations)
    # Read one observation
    logger.info('Waiting for one observation...')
    while not rospy.is_shutdown():  # @UndefinedVariable
        if Global.observations is not None:
            break
        rospy.sleep(sleep)  # @UndefinedVariable
    # Close this subscriber
    subscriber.unregister()
    return Global.observations

