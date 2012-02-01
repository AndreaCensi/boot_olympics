''' Utilities for creating parallel jobs using Compmake. '''

from .. import np, logger, contract
from .batch_config import *
from .batch_manager import *

from compmake import comp

first_person = ['pdff2sb',
               'pdff2no',
               'mp4f2sb',
               'mp4f2no']

default_expl_videos = ['pdfz2sb',
                      'pdff2sb',
                      'pdfz0sb',
                      'pdfz2no',
                      'pdff2no',
                      'pdfz0no',
                      'mp4z2sb',
                      'mp4f2sb',
                      'mp4z0sb',
                      'mp4z2no',
                      'mp4f2no',
                      'mp4z0no']

# TODO: check not repeated
default_servo_videos = ['avisrvz0'] + first_person
default_servonav_videos = ['avisrvz0', 'mp4z0nosrv'] + first_person


from .batch_learn import *

