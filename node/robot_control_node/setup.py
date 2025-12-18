from setuptools import setup

package_name = 'robot_control_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zvcxzvcx201-design',
    maintainer_email='zvcxzvcx201@gmail.com',
    description='Robot control node - MoveIt interconnection layer with motion queue',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_control_node = robot_control_node.robot_control_node:main',
        ],
    },
)
