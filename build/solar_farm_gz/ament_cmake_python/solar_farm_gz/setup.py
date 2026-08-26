from setuptools import find_packages
from setuptools import setup

setup(
    name='solar_farm_gz',
    version='0.1.0',
    packages=find_packages(
        include=('solar_farm_gz', 'solar_farm_gz.*')),
)
