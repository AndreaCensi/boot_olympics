


from .main import BOM



class CmdNuisLog(BOM.get_sub()):
    ''' Runs a log through a nuisance.

        Example:
            bom -d sets/bv1bds2 nuislog -r Se0Vrb1co -e Yrl1Se0Vrb1co
    '''

    cmd = 'nuislog'

    def define_program_options(self, params):
        params.add_string('equiv', short="-e", help="EquivRobot ID")
        params.add_string('robot', short='-r')

    def go(self):
        options = self.get_options()
        data_central = self.get_parent().get_data_central()


        id_equiv = options.equiv
        id_robot = options.robot
        from boot_manager.meat.nuislog import task_nuislog
        task_nuislog(data_central, id_equiv=id_equiv, id_robot=id_robot)
