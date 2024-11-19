from setuptools import find_packages
from setuptools import setup

setup(
    name='moveit_controller',
    version='0.0.0',
    packages=find_packages(
        include=('moveit_controller', 'moveit_controller.*')),
)
