
from bootstrapping_olympics import LogsFormat, BootOlympicsConstants
from conf_tools import GlobalConfig
import numpy as np
from quickapp import QuickMultiCmdApp

from ..meat import DataCentral, DirectoryStructure
import os
import contracts


class BOM(QuickMultiCmdApp):
    """ Main program for running bootstrapping experiments. """


    def define_multicmd_options(self, params):

        params.add_string("boot_root", short="-d", default=None,
                          help="Root directory with logs, config, etc.")

        params.add_string_list("extra_conf_dirs", short='-c', default=[],
                          help='Adds an extra config dir.')

        params.add_string_list('extra_log_dirs', short="-l", default=[],
                          help='Adds an extra directory storing logs.')

        params.add_string('seterr', default="warn",
                          help="Sets np.seterr. "
                          "Possible values: ignore, warn, raise, print, log")

        params.add_flag('contracts')

        available = LogsFormat.formats.keys()
        params.add_string("logformat",
                          default=BootOlympicsConstants.DEFAULT_LOG_FORMAT,
                          help="Choose format for writing logs in %s"
                            % str(available))

    def initial_setup(self):
        options = self.get_options()

        if not options.contracts:
            contracts.disable_all()

        if options.boot_root is None:
            options.boot_root = DirectoryStructure.DEFAULT_ROOT
            self.info('Using %r as default root directory '
                        '(use -d <dir> to change)' % options.boot_root)


        np.seterr(all=options.seterr)
        # underflow is very common in all libraries (e.g. matplotlib)
        np.seterr(under='warn')

        data_central = DataCentral(options.boot_root)
    
        GlobalConfig.global_load_dir('default')  # need skins
        for dirname in options.extra_conf_dirs:
            GlobalConfig.global_load_dir(dirname)

        cwd_config = os.path.join(os.getcwd(), 'config')
        if os.path.exists(cwd_config):
            GlobalConfig.global_load_dir(cwd_config)
    
        dir_structure = data_central.get_dir_structure()
        dir_structure.set_log_format(options.logformat)
        for dirname in options.extra_log_dirs:
            dir_structure.add_log_directory(dirname)

        self._data_central = data_central

    def get_data_central(self):
        return self._data_central


manager_main = BOM.get_sys_main()

if __name__ == '__main__':
    manager_main()
