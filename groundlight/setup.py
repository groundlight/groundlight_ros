from setuptools import find_packages, setup
import glob

package_name = 'groundlight'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', glob('msg/*.msg')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tim Huff',
    maintainer_email='tim@groundlight.ai',
    description='Build computer vision systems from natural language with Groundlight',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_query_handler = groundlight.image_query_handler:main',
            'camera = groundlight.camera:main',
        ],
    },
)
