from glob import glob

from setuptools import find_packages, setup

package_name = 'rtr_navigate'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Justin Albrecht',
    maintainer_email='justin.albrecht@nist.gov',
    description='Example task: drive to a target pose, built on rtr_core.',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigate = rtr_navigate.executor:main',
            'sim = rtr_navigate.sim:main',
            'client = rtr_navigate.client:main',
        ],
    },
)
