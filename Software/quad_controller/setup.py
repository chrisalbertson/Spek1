#!/usr/bin/env python

from distutils.core import setup

setup(name='quad_controller',
      version='0.1',
      description='Controller for SpotMicro robot',
      author='Chris Albertson',
      author_email='albertson.chris@gmail.com',
      py_modules=['main',
                  'config',
                  'footpath',
                  'robot',
                  'sm_kinematics',
                  'user_interface',
                  'joints',
                  'gui',
                  'setup'],
      )
