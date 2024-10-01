from setuptools import find_packages
from setuptools import setup

setup(
    name='messages_fr3',
    version='1.0.0',
    packages=find_packages(
        include=('messages_fr3', 'messages_fr3.*')),
)
