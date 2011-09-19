import os
from setuptools import setup, find_packages

version = "0.1"

description = """""" 

    
long_description = description  

setup(name='BootstrappingOlympics',
      url='http://github.com/AndreaCensi/boot_olympics/',
      
      description=description,
      long_description=long_description,
      keywords="",
      license="",
      
      classifiers=[
        'Development Status :: 4 - Beta',
        # 'Intended Audience :: Developers',
        # 'License :: OSI Approved :: GNU Library or Lesser General Public License (LGPL)',
        # 'Topic :: Software Development :: Quality Assurance',
        # 'Topic :: Software Development :: Documentation',
        # 'Topic :: Software Development :: Testing'
      ],

      version=version,
#      download_url='http://github.com/GITHUB_USER/GITHUB_PROJECT/tarball/%s' % version,
      
      package_dir={'':'src'},
      packages=find_packages('src'),
      install_requires=[ 'PyYAML', 'RepRep', 'PyContracts', 'ConfTools'],
      tests_require=['nose'],
      entry_points={
         'console_scripts': [
           'boot_olympics_print_config = '
                'bootstrapping_olympics.programs.print_config:main',
           'boot_olympics_create_launch_agent_robot = '
                'bootstrapping_olympics.ros_scripts.create_launch_agent_robot:main',
            'boot_olympics_create_launch_all = '
                'bootstrapping_olympics.ros_scripts.create_launch_all:main',
            'boot_gui = '
                'boot_gui.boot_gui:main',
            'boot_learn = '
                'bootstrapping_olympics.ros_scripts.log_learn.main:main',
            'boot_hdf2bag = '
                'bootstrapping_olympics.ros_scripts.hdf2bag.main:main',

        ]
      }          
)

