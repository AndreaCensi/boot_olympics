''' Command-line interface to the functions in "meat". '''
from .. import  np, contract

from .common import *
from .commands_list import *

import logging
logger = logging.getLogger("BO.cmdline")

# TODO: put in utils/
import optparse
from optparse import IndentedHelpFormatter


class LenientOptionParser(optparse.OptionParser):

    def parse_args(self, args):
        self.arguments = list(args)
        return optparse.OptionParser.parse_args(self, args)

    def error(self, msg):
        #msg = '%s: %s' % (self.get_prog_name(), msg)
        msg += ('\nArguments: %s %s' %
                (self.get_prog_name(), " ".join(self.arguments)))
        raise UserError(msg)


def OptionParser(prog, usage):
    formatter = IndentedHelpFormatter(
                 indent_increment=2,
                 max_help_position=80,
                 width=100,
                 short_first=1)

    return LenientOptionParser(prog=prog, formatter=formatter, usage=usage)


from . import predict
from . import learn
from . import list_agents
from . import list_logs
from . import list_robots
from . import list_states
from . import c_simulate
from . import servo
from . import servonav
from . import video
from . import batch
from . import servo_stats
from . import clean_states
from . import clean_simulations


from .main import *

