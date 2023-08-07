from setuptools import setup
import os
from glob import glob

package_name = 'pointcloud_segmentation_r2'

def recursive_data_files(src_directory, dest_directory):
    paths = []
    for (path, directories, filenames) in os.walk(src_directory):
        rel_path = os.path.relpath(path, src_directory)
        dest_path = os.path.join(dest_directory, rel_path)
        files = [os.path.join(path, filename) for filename in filenames]
        paths.append((dest_path, files))
    return paths

data_files = recursive_data_files('models', os.path.join('share', package_name, 'models'))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files= data_files + [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
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
            'pointcloud_segmentation = pointcloud_segmentation_r2.pointcloud_segmentation:main',
        ],
    },
)
