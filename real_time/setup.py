from setuptools import find_packages, setup
import os 
from glob import glob
package_name = 'real_time'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('data/*')),
        (os.path.join('share', package_name), glob('natnet/*')),
        (os.path.join('share', package_name), glob('model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotics20',
    maintainer_email='rotem.atri@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'natnet_move_joint=real_time.natnet_move_joint:main',
        ],
    },
)
