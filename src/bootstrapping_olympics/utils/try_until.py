from .. import logger

def try_until_done(function): 
    while True:
        try:
            function()
            break
        except KeyboardInterrupt:
            logger.info('Caught CTRL-C, retrying.')
            continue   
