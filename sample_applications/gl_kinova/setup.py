from setuptools import find_packages, setup

package_name = 'gl_kinova'

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
    description='Sample application using Groundlight with a Kinova arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo = gl_kinova.kinova_demo:main',
        ],
    },
)
