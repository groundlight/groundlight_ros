from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gl_image_query'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tim Huff',
    maintainer_email='tim@groundlight.ai',
    description='Perform natural language image queries with Groundlight',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_server = gl_image_query.action_server:main',
            'rviz_markers = gl_image_query.rviz_markers:main',
            'hello_world = gl_image_query.hello_world:main',
            'webcam_example = gl_image_query.webcam_example:main',
        ],
    },
)
