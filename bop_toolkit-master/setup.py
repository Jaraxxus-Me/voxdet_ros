#!/usr/bin/env python
from distutils.core import setup, Extension
from setuptools import find_packages

setup(
    name='bop_toolkit',
    version='0.1dev',
    author='Thomas Hodan',
    packages=find_packages('.'),
    package_dir={'bop_toolkit_lib': 'bop_toolkit_lib'},
    description= 'A Python toolkit of the BOP benchmark for 6D object pose estimation',
    long_description='',
)
