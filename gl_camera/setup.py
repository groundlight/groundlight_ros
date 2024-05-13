from setuptools import find_packages, setup

package_name = 'gl_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tim Huff',
    maintainer_email='tim@groundlight.ai',
    description='Provides a services to grab frames from existing ROS camera streams',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_server = gl_camera.camera_server:main',
            'webcam = gl_camera.webcam:main',
        ],
    },
)
