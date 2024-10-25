from setuptools import find_packages, setup

package_name = 'turtle_brick'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/show_turtle.launch.xml',
                                   'launch/show_turtle.launch.py',
                                   'launch/run_turtle.launch.xml',
                                   'launch/turtle_arena.launch.xml',
                                   'urdf/turtle.urdf.xacro',
                                   'config/view_robot.rviz',
                                   'config/turtle.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lbos7',
    maintainer_email='loganstuartboswell@gmail.com',
    description='A ROS 2 package for controlling a robot to catch a brick in rviz2',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_robot = turtle_brick.turtle_robot:main',
            'arena = turtle_brick.arena:main'
        ],
    },
)
