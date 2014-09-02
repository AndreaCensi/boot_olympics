from conf_tools import ConfigMaster, ObjectSpec
from contracts import contract

__all__ = [
    'get_bootbatch_config', 
    'get_conftools_bootbatchsets',
]

class BatchConfigMaster(ConfigMaster):
    def __init__(self):
        ConfigMaster.__init__(self, 'batchconfig')
        self.add_class('sets', '*.sets.yaml', check_valid_set_config)

    def get_default_dir(self):
        from pkg_resources import resource_filename  # @UnresolvedImport
        return "boot_manager.configs"

get_bootbatch_config = BatchConfigMaster.get_singleton

@contract(returns=ObjectSpec)
def get_conftools_bootbatchsets():
    return get_bootbatch_config().sets

def check_valid_set_config(struct):
    pass


