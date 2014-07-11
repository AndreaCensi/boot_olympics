from ..meat import create_video
from .main import BOM


class CmdVideo(BOM.get_sub()):
    """ Creates a video from one episode. """
    cmd = 'video'

    def define_program_options(self, params):
        params.add_string('robot', short='-r')
        params.add_string('episode', short="-e", help="Episode ID", default="")
        params.add_string('model', short="-m", default='boot_log2movie',
                      help="Procgraph model used for visualization. ")


    def go(self):
        options = self.get_options()
        data_central = self.get_parent().get_data_central()
        create_video(data_central=data_central,
                  id_agent=options.agent,
                  id_robot=options.robot,
                  id_episode=options.episode,
                  model=options.model)
