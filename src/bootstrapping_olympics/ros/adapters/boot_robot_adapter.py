from .. import (BootstrappingCommandsResponse, BootstrappingObservations,
    BootstrappingCommands, rospy, np)
from ...configuration import check_valid_robot_config
from ...interfaces import ObsKeeper
from ...programs.manager.meat import DataCentral
from ...utils import check_parameters
from ..ros_logs import observations2ros
from ..ros_script_utils import RospyLogger
import time


class BootRobotAdapter:
    def __init__(self, id_robot, robot, maximum_interval=2, sleep=0.1,
                 check_valid_values=True):
        self.robot = robot
        self.maximum_interval = maximum_interval
        self.sleep = sleep
        self.logger = RospyLogger('robot_adapter')

        self.publisher = rospy.Publisher('~observations',
                                         BootstrappingObservations,
                                         latch=True)
        # Start service
        rospy.Service('~commands', BootstrappingCommands,
                      self.commands_request)

        self.episode = self.robot.new_episode()

        self.publish = True # XXX

        self.keeper = ObsKeeper(boot_spec=robot.get_spec(),
                                id_robot=id_robot)

        self.check_valid_values = check_valid_values

        self.obs_spec = self.robot.get_spec().get_observations()
        self.cmd_spec = self.robot.get_spec().get_commands()

    def publish_observations(self):
        obs = self.robot.get_observations()

        if self.check_valid_values:
            self.obs_spec.check_valid_value(obs.observations)
            self.cmd_spec.check_valid_value(obs.commands)

        observations = self.keeper.push(timestamp=obs.timestamp,
                                   observations=obs.observations,
                                   commands=obs.commands,
                                   commands_source=obs.commands_source,
                                   id_episode=self.episode.id_episode,
                                   id_world=self.episode.id_environment)

        msg = observations2ros(robot_spec=self.robot.get_spec(),
                                  obs=observations)

        self.publisher.publish(msg)
        self.logger.info('Published information.')

    def commands_request(self, req):
        self.logger.info('Received request.')
        commands = np.array(req.commands)
        sender = req.sender
        self.robot.set_commands(commands=commands,
                                commands_source=sender)
        self.publish = True
        return BootstrappingCommandsResponse(True)

    def loop(self):
        self.logger.info('Loop started')
        last_observations_sent = 0

        while not rospy.is_shutdown():
            now = time.time()
            publish = False
            if self.publish:
                self.publish = False
                publish = True
            if now > last_observations_sent + self.maximum_interval:
                last_observations_sent = now
                publish = True
            if publish:
                self.publish_observations()

            self.logger.info('Sleeping...')

            #self.sleep = 1
            rospy.sleep(self.sleep)


def boot_robot_adapter_main(params):
    check_parameters(params, required=['robot_spec', 'root'],
                             optional=['maximum_interval', 'sleep'])

    data_central = DataCentral(params['root'])
    bo_config = data_central.get_bo_config()

    sleep = params.get('sleep', 0.0)
    maximum_interval = params.get('maximum_interval', 2)
    robot_spec = params['robot_spec']
    check_valid_robot_config(robot_spec)
    robot = bo_config.robots.instance_spec(robot_spec) #@UndefinedVariable 

    adapter = BootRobotAdapter(id_robot=robot_spec['id'],
                               robot=robot,
                              maximum_interval=maximum_interval,
                              sleep=sleep)
    adapter.loop()
