from setuptools import find_packages, setup

package_name = 'test_turtle'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cycle_exe=test_turtle.cycle:main",
            "spiral_exe=test_turtle.spiral:main",
            "spiral_uniform_exe=test_turtle.spiral_uniform:main",
            "reset_exe=test_turtle.reset:main",
            "circle_timer_exe=test_turtle.circle_timer:main",
            "circle_exe=test_turtle.circle:main",
            "circles_exe=test_turtle.circles:main",
            "line_exe=test_turtle.line:main",
            "teleport_exe=test_turtle.teleport:main",
            "drone_exe=test_turtle.drone:main",
        ],
    },
)
