from .main import BOM


class CmdServoStats(BOM.get_sub()):
    """ Computes statistics for a servo experiment. """
    cmd = 'servo_stats'

    def define_program_options(self, params):
        params.add_string('agent', short="-a")
        params.add_string('robot', short='-r')

    def go(self):
        options = self.get_options()
        data_central = self.get_parent().get_data_central()

        id_agent = options.agent
        id_robot = options.robot

        from boot_manager.meat.servo.summaries import servo_stats_summaries
        from boot_manager.meat.servo.report import servo_stats_report
    
        summaries = servo_stats_summaries(data_central, id_agent, id_robot)
        servo_stats_report(data_central, id_agent, id_robot, summaries)
