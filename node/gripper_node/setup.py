from setuptools import setup

package_name = 'gripper_node'

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
    description='Franka gripper control node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_node = gripper_node.gripper_node:main',
        ],
    },
)
