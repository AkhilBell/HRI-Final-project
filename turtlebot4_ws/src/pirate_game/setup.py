from setuptools import setup

package_name = 'pirate_game'

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
    maintainer='ubuntu',
    maintainer_email='mp2334@cornell.edu',
    description='Pirate Treasure Island Game for Turtlebot 4',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This line registers the command "ros2 run pirate_game test_nav"
            'test_nav = pirate_game.simple_nav_test:main',
            # This line registers the command "ros2 run pirate_game pirate_game"
            'pirate_game = pirate_game.pirate_game:main',
            # This line registers the command "ros2 run pirate_game movement_test"
            'movement_test = pirate_game.movement_test:main',
            # This line registers the command "ros2 run pirate_game tts_test"
            'tts_test = pirate_game.tts_test:main',
            # This line registers the command "ros2 run pirate_game speech_test"
            'speech_test = pirate_game.speech_test:main',
        ],
    },
)
