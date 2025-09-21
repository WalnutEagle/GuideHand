from setuptools import setup
import os
from glob import glob

package_name = 'assistive_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'blobconverter'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.com',
    description='OAK-D perception node for handover.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = assistive_perception.perception_node:main'
        ],
    },
)