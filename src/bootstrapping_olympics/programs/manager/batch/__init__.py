''' Utilities for creating parallel jobs using Compmake. '''

from .. import np, logger, contract
from .batch_config import *
from .batch_manager import *

pdfs_first = ['pdff2sb',
               'pdff2no']


first_person = [
               'mp4f2sb',
               'mp4f2no']

pdfs = ['pdfz2sb',
                      'pdff2sb',
                      'pdfz0sb',
                      'pdfz2no',
                      'pdff2no',
                      'pdfz0no']

default_expl_videos = [
                      'mp4z2sb',
#                      'mp4f2sb',
#                      'mp4z0sb',
#                      'mp4z2no',
#                      'mp4f2no',
                      'mp4z0no']

# TODO: check not repeated

default_servo_videos = ['mp4f2sr', 'mp4f2no'] # 'mp4z0sr', 
default_servonav_videos = ['mp4f2sr', 'mp4f2no'] # 'mp4z0sr', 


from .batch_learn import *

