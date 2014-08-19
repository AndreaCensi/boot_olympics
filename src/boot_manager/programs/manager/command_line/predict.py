from .main import BOM


class CmdPredict(BOM.get_sub()):
    '''Simulate the interaction of an agent and a robot. '''
    cmd = 'predict'

    def define_program_options(self, params):
        params.add_string("agent", short="-a", help="Agent ID")
        params.add_string("robot", short='-r', help="Robot ID")

        params.add_float("--interval_print", default=5,
                      help='Frequency of debug messages.')

        params.add_string_list("plugin", default=[],
                      help="Run the specified plugin model during "
                           "learning. (eg, visualization)")

    def go(self):
        options = self.get_options()
        
        data_central = self.get_parent().get_data_central()

        id_agent = options.agent
        id_robot = options.robot
        from boot_manager.meat.predict import task_predict
        stats = task_predict(data_central,
             id_agent=id_agent,
             id_robot=id_robot,
             live_plugins=options.plugin)

        from boot_manager.meat.predict import predict_report
        predict_report(data_central, id_agent, id_robot, stats)

