from .main import BOM
from boot_manager.programs.manager.batch import batch_process_manager
from quickapp import QuickApp

__all__ = []

class CmdBatch(QuickApp, BOM.get_sub()):
    """ Batch learning experiments """
    cmd = 'batch'
    def define_options(self, params):
        params.accept_extra()  # remainder arguments
    
    def define_jobs_context(self, context):
        data_central = self.get_parent().get_data_central()
        which_sets = self.get_options().get_extra()

        self.info('which_sets: %s' % str(which_sets))
        
        batch_process_manager(context=context,
                              data_central=data_central,
                              which_sets=which_sets)
