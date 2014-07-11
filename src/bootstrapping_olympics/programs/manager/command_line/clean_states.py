from bootstrapping_olympics import logger

from .main import BOM


class CmdCleanStates(BOM.get_sub()):
    ''' Cleans agents states. '''

    cmd = 'clean-states'

    def define_program_options(self, params):
        params.add_string('agent', short='-a')
        params.add_string('robot', short='-r')

    def go(self):
        options = self.get_options()
        data_central = self.get_parent().get_data_central()

        id_agent = options.agent
        id_robot = options.robot

        db = data_central.get_agent_state_db()

        # TODO: check we know agent and robot?

        if not db.has_state(id_agent=id_agent, id_robot=id_robot):
            logger.info('The state has already been cleaned.')
        else:
            logger.info('Cleaning state for %r/%r...' % (id_agent, id_robot))
            db.delete_state(id_agent=id_agent, id_robot=id_robot)
