from glob import glob

from setuptools import find_packages, setup

package_name = 'rtr_count'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Justin Albrecht',
    maintainer_email='justin.albrecht@nist.gov',
    description='Example task: count to a target number, built on rtr_core.',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'count = rtr_count.executor:main',
            'client = rtr_count.client:main',
        ],
    },
)
