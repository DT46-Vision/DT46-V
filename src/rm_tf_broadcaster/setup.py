from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rm_tf_broadcaster'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kielas',
    maintainer_email='c1470759@outlook.com',
    description='Publish enemy target position to tf tree and PoseStamped topic',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'enemy_tf_node = rm_tf_broadcaster.enemy_tf_node:main',
        ],
    },
)
