import os
from setuptools import setup, find_packages

version = "1.0"

description = """"""

long_description = description

setup(name='BootOlympics',
      url='http://github.com/AndreaCensi/boot_olympics/',

      description=description,
      long_description=long_description,
      keywords="agents learning bootstrapping",
      license="LGPL",

      classifiers=[
        'Development Status :: 4 - Beta',
        'License :: OSI Approved :: GNU Library or Lesser General Public License (LGPL)',
      ],

      version=version,
      download_url='http://github.com/AndreaCensi/boot_olympics/tarball/%s' % version,

      package_dir={'':'src'},
      packages=find_packages('src'),
      install_requires=[
	'nose',
        'PyYAML',
        'PyContracts>=1.2,<2',
        'ConfTools>=1.0,<2',
        'RepRep>=1.0,<3',
#        'compmake>=1.0,<2',
      ],
      extras_require={},

      setup_requires=['nose>=1.0'],
      tests_require=['nose>=1.0', 'rudolf', 'nose-progressive', 'nose-cov'],

      entry_points={
         'console_scripts': [
           'boot_olympics_print_config = '
                'bootstrapping_olympics.programs.print_config:main',
#           'boot_olympics_create_launch_agent_robot = '
#                'bootstrapping_olympics.ros_scripts.create_launch_agent_robot:main',
#            'boot_olympics_create_launch_all = '
#                'bootstrapping_olympics.ros_scripts.create_launch_all:main',
            'boot_olympics_gui = '
                'boot_gui.boot_gui:main',
            'boot_olympics_manager = '
                'bootstrapping_olympics.programs.manager:manager_main',
            'bom = '
                'bootstrapping_olympics.programs.manager:manager_main',
            'boot_olympics_hdf2bag = '
                'bootstrapping_olympics.programs.hdf2bag.main:main',
            'boot_olympics_rosbag2h5 = '
                'bootstrapping_olympics.programs.rosbag2h5.main:main',

        ]
      },
      test_suite='nose.collector'
)

