from setuptools import find_packages, setup

package_name = 'groundlight_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'groundlight_ros'],
    # packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', ['srv/AddTwoInts.srv']),
    ],

    install_requires=[
        'setuptools',
        'groundlight', # the Groundlight Python SDK
        ],
    zip_safe=True,
    maintainer='Tim Huff',
    maintainer_email='tim@groundlight.ai',
    description='Build computer vision systems from natural language with Groundlight',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_query_handler = groundlight_ros.image_query_handler:main',
            'camera = groundlight_ros.camera:main',
            'add_two_ints_server = groundlight_ros.add_two_ints_server:main',
        ],
    },
)
