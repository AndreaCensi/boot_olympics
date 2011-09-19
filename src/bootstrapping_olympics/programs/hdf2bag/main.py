from . import logger
from optparse import OptionParser
from procgraph import pg
import os
import sys
import traceback
from glob import glob
import yaml

usage = """

    boot_hdf2bag <directory>
    
Creates a .bag file.
  
"""  

def hdf2bag(pargs):
    parser = OptionParser(usage=usage)
    parser.disable_interspersed_args()

    (options, args) = parser.parse_args(pargs) #@UnusedVariable
    
    if len(args) != 1:
        msg = 'I expect only one filename.'
        raise Exception(msg) 
    

    basedir = args[0]
    global_config_file = os.path.join(basedir, 'robot_info.yaml')
    if not os.path.exists(global_config_file):
        raise Exception('Configuration file %r not found.' % global_config_file)
    
    global_config = yaml.load(open(global_config_file))
    print global_config.__repr__()
    for hdf_filename in glob(os.path.join(basedir, '*.h5')):
        bag_filename = os.path.splitext(hdf_filename)[0] + '.bag'
        id_episode = os.path.basename(os.path.splitext(hdf_filename)[0])
        pg('hdf2bag_conversion',
           dict(hdf=hdf_filename,
                bag=bag_filename,
                id_robot=global_config['id_robot'],
                id_actuators=global_config['id_actuators'],
                id_sensors=global_config['id_sensors'],
                id_episode=id_episode,
                id_environment=global_config['id_environment'],
                commands_spec=global_config['commands_spec']))

# TODO: use generic function
def main():
    try:
        hdf2bag(sys.argv[1:])
        sys.exit(0)
    except Exception as e:
        logger.error(str(e))
        logger.error(traceback.format_exc())
        sys.exit(-2) 
    
if __name__ == '__main__':
    main()
