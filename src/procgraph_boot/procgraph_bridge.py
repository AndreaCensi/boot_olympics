from bootstrapping_olympics.interfaces import LivePlugin
from procgraph.core.registrar import default_library, Library
from procgraph.core.model_loader import pg_look_for_models
from contracts import contract
from bootstrapping_olympics import logger

__all__ = ['ProcgraphBridge']


class ProcgraphBridge(LivePlugin):
    """ 
        Bridge to allow pushing data through a procgraph model.
        
        The model must have the config "basename", "id_agent", "id_robot"
        and the inputs "agent", "robot", "obs".
    """

    @contract(procgraph_extra_modules='list(str)', suffix='str')
    def __init__(self, procgraph_code, procgraph_extra_modules, suffix):
        """ 
            :param suffix: Suffix for the video
            :param code: Procgraph model/params (array of string, dict) 
        """
        self.suffix = suffix
        self.code = procgraph_code  # TODO: check

        for module in procgraph_extra_modules:  # XXX: maybe module:model
            __import__(module)

    def init(self, init_data):
        data_central = init_data['data_central']
        id_agent = init_data['id_agent']
        id_robot = init_data['id_robot']
        ds = data_central.get_dir_structure()
        basename = ds.get_video_basename(id_robot=id_robot,
                                         id_agent=id_agent,
                                         id_episode=None)
        if self.suffix:
            basename += '-%s' % self.suffix

        library = Library(default_library)
        pg_look_for_models(library)

        # load standard components
        import procgraph.components  # @UnusedImport

        # Give config passed by user
        config = dict(**self.code[1])
        
        can_provide = dict(**init_data)
        can_provide['basename'] = basename

        # Finding what configuration it wants
        block_type = self.code[0]
        # This is the config that it can accept
        can_accept = [x.variable for x in
                      library.get_generator_for_block_type(block_type).config]
        # Now give extra
        for k, v in can_provide.items():
            if k in can_accept:
                config[k] = v

        self.model = library.instance(block_type, name='bridge',
                                      config=config)
        self.model.init()
        
        self.first_time = True
        
    @contract(update_data='dict')
    def update(self, update_data): 
        timestamp = update_data['obs']['timestamp']
        for k, v in update_data.items():
            if not self.model.is_valid_input_name(k):
                if self.first_time:
                    logger.info('%r: not sending' % k)
            else:
                if self.first_time:
                    logger.info('%r: sending' % k)
                self.model.from_outside_set_input(k, v, timestamp)
        self.first_time = False
        
        while self.model.has_more():
            self.model.update()

    def finish(self):
        self.model.finish()
