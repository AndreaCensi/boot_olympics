__version__ = '1.0'

import sys
import traceback


class UserError(Exception):
    pass


def wrap_script_entry_point(function, logger):
    try:
        ret = function(sys.argv[1:])
        if ret is None:
            ret = 0
        sys.exit(ret)
    except UserError as e:
        logger.error(str(e))
        sys.exit(1)
    except Exception as e:
        logger.error(traceback.format_exc())
        sys.exit(-55)
