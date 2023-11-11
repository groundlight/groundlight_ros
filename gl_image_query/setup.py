from setuptools import find_packages, setup

package_name = 'gl_image_query'

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
    description='Peform natural language image queries with Groundlight',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = gl_image_query.image_query_server:main',
            'action_server = gl_image_query.image_query_action_server:main',
            'client = gl_image_query.sample_groundlight_client:main',
        ],
    },
)
