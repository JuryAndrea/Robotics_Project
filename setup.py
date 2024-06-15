from setuptools import setup

package_name = 'robomaster_proj'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotics23',
    maintainer_email='altric@usi.ch',
    description='Robomaster project',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robomaster_node = robomaster_proj.robomaster_node:main',
            'robomapper_node = robomaster_proj.robomapper_node:main',
            'robomapper_node2 = robomaster_proj.robomapper_node_unified:main',
            'robomaster_old = robomaster_proj.robomaster_1approach_node:main'
        ],
    },
)
