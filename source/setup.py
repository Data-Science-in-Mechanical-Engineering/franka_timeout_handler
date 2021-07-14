from setuptools import setup, find_packages
import sys

if sys.version_info < (3, 0):
  sys.exit('Python < 3.0 is not supported')

setup(
  name        = 'franka_real_time',
  version     = '0.0.1', # TODO: might want to use commit ID here
  packages    = find_packages(),
)