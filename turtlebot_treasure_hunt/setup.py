from setuptools import setup

package_name = 'turtlebot_treasure_hunt'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/treasure_hunt.launch.py']),
        ('share/' + package_name + '/config', ['config/game_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='TurtleBot4 Treasure Hunt Game',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'treasure_hunt_game = turtlebot_treasure_hunt.treasure_hunt_game:main',
        ],
    },
)
