from setuptools import find_packages, setup
import os

package_name = 'gl_turtlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', 
         [os.path.join('launch', file) for file in os.listdir('launch') if file.endswith('.py')]),
        (f'share/{package_name}/config', 
         [os.path.join('config', file) for file in os.listdir('config')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tim',
    maintainer_email='tim@groundlight.ai',
    description='A demonstration of Groundlight AI on a Turtlebot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo = gl_turtlebot.demo:main',
        ],
    },
)
