from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'mp_eval'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'assets'), glob(os.path.join('assets', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='tbishnoi@torontomu.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mp_eval = mp_eval.mp_eval:main',
            'metrics_collector = mp_eval.metrics_collector:main',
            'scene_viewer = mp_eval.scene_viewer:main',
        ],
    },
)