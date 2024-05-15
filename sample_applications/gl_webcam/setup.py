from setuptools import find_packages, setup
import os

package_name = 'gl_webcam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', 
         [os.path.join('launch', file) for file in os.listdir('launch') if file.endswith('.py')]),
        (f'share/{package_name}/config', 
         [os.path.join('config', file) for file in os.listdir('config')]),
        (f'share/{package_name}/urdf', 
        [os.path.join('urdf', file) for file in os.listdir('urdf')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tim',
    maintainer_email='tim@groundlight.ai',
    description='A simple example of using groundlight_ros with a webcam.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo = gl_webcam.demo:main',
            'joint_publisher = gl_webcam.joint_publisher:main',
            'frustum_publisher = gl_webcam.frustum_publisher:main',
            'image_drawer = gl_webcam.image_drawer:main',
        ],
    },
)
