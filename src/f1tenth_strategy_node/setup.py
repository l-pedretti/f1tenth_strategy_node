from setuptools import setup

package_name = 'f1tenth_strategy_node'

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
    maintainer='LeonardoPedretti-GiuliaCasarini',
    maintainer_email='you@email.com',
    description='Strategy node based on a finite state machine for F1Tenth car',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
            'console_scripts': [
                    'obstacle = f1tenth_strategy_node.publisher_obstacle:main',
                    'car = f1tenth_strategy_node.publisher_car:main',
                    'fsm = f1tenth_strategy_node.subscriber_fsm:main',
                    'fsm_reader = f1tenth_strategy_node.sub_fsm:main',
            ],
    },
)
