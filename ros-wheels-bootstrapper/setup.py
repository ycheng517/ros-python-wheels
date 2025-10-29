from setuptools import setup, find_packages

setup(
    name='ros-wheels-bootstrapper',
    version='0.1.2',
    description='A package to bootstrap the ROS environment for wheels by setting LD_LIBRARY_PATH.',
    author='user',
    author_email='user@example.com',
    license='Apache-2.0',
    packages=find_packages() + [''],
    package_data={'': ['ros_wheels_bootstrapper.pth']},
)
