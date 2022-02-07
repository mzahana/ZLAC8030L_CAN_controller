# -*- coding: utf-8 -*-

# Learn more: https://github.com/mzahana/ZLAC80380L_CAN_controller.git
# Reference for Python package structuring (https://www.pythonforthelab.com/blog/how-create-setup-file-your-project/)

from setuptools import setup, find_packages


with open('README.md') as f:
    readme = f.read()

with open('LICENSE') as f:
    license = f.read()

setup(
    name='ZLAC80380L_CAN_controller',
    version='0.1.0',
    description='A Python package for CANopen communication with the ZLAC8030L motor driver.',
    long_description=readme,
    author='Mohamed Abdelkader',
    author_email='mohamedashraf123@gmail.com',
    url='https://github.com/mzahana/ZLAC80380L_CAN_controller.git',
    license=license,
    packages=find_packages(exclude=('tests', 'docs')),
    entry_points={
        'console_scripts': [
            'zlac8030l_test= ZLAC80380L_CAN_controller.__main__:main',
        ]
    },
    install_requires=[
        "canopen;python_version>='3.6'",
    ]
)