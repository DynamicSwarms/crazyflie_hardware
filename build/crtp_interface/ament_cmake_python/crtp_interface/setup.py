from setuptools import find_packages
from setuptools import setup

setup(
    name='crtp_interface',
    version='0.0.0',
    packages=find_packages(
        include=('crtp_interface', 'crtp_interface.*')),
)
