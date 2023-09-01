from setuptools import setup
import os
from glob import glob

package_name = 'camera_based_semantic_grid_mapping_r2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ACDC',
    maintainer_email='acdc@ika.rwth-aachen.de',
    description='This package performs semantic grid mapping using inverse perspective mapping on segmented camera images.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_grid_mapping = camera_based_semantic_grid_mapping_r2.semantic_grid_mapping:main',
        ],
    },
)
