from bootstrapping_olympics import get_boot_config
from bootstrapping_olympics.interfaces.observations import ObsKeeper
from bootstrapping_olympics.misc.interaction import iterate_robot_observations
from procgraph import Block, IteratorGenerator


__all__ = ['RawlogBootReader']


class RawlogBootReader(IteratorGenerator):
    Block.alias('rawlog_boot_reader')
    Block.config('id_robot', dtype='str')
    Block.config('id_rawlog', 'Rawlog description (used as id_episode)',
                 dtype='str')
    Block.config('rawlog', 'Rawlog instance', dtype='isinstance(Rawlog)')
    Block.output('boot_observations')

    def init_iterator(self):
        boot_config = get_boot_config()
        robot = boot_config.robots.instance(self.config.id_robot)
        orig_robot = robot.get_inner_components()[-1]

        from rosstream2boot.library.ros_robot import ROSRobot
        if not isinstance(orig_robot, ROSRobot):
            from contracts.interface import describe_type
            msg = 'Expected ROSRobot, got %s' % describe_type(robot)
            raise ValueError(msg)

        orig_robot.read_from_rawlog(self.config.rawlog)
        id_environment = 'not-specified'  # rawlog.get_id_environment()

        boot_spec = robot.get_spec()

        keeper = ObsKeeper(boot_spec=boot_spec, id_robot=self.config.id_robot,
                           check_valid_values=False)

        for obs in iterate_robot_observations(robot, sleep=0):
        # print('robot_pose: %s' % obs.robot_pose)
            # print('got %s' % obs['timestamp'])
            boot_observations = keeper.push(timestamp=obs.timestamp,
                                            observations=obs.observations,
                                            commands=obs.commands,
                                            commands_source=obs.commands_source,
                                            id_episode=self.config.id_rawlog,
                                            id_world=id_environment)
            yield 'boot_observations', obs.timestamp, boot_observations

