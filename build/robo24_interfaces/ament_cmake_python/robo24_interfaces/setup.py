from setuptools import find_packages
from setuptools import setup

setup(
    name='robo24_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('robo24_interfaces', 'robo24_interfaces.*')),
)
