from setuptools import find_packages, setup

package_name = 'turtlesim_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mia',
    maintainer_email='mia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "vel_controller_node=turtlesim_py_pkg.vel_controller:main",
            "go_to_location_node=turtlesim_py_pkg.go_to_loc:main",
            "spawn_turtle_node=turtlesim_py_pkg.spawn_turtle:main",
            "find_go_to_location_node=turtlesim_py_pkg.find_go_to_loc:main",
            "kill_turtle_node=turtlesim_py_pkg.kill_turtle:main",
            "find_and_kill_node=turtlesim_py_pkg.find_and_kill_test:main"
            
        ],
    },
)
