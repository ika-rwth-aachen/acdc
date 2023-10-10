from setuptools import setup
import os
from glob import glob

package_name = 'image_segmentation_r2'

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
        (os.path.join('share', package_name, 'models'), glob('models/*.*')), 
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Till Beemelmanns',
    maintainer_email='till.beemelmanns@rwth-aachen.de',
    description='This package performs semantic image segmentation on camera images.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_segmentation = image_segmentation_r2.image_segmentation:main',
        ],
    },
)
