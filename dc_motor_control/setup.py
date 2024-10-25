from setuptools import find_packages, setup

package_name = 'dc_motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), ('share/' + package_name, ['config/params.yaml']),
        ('share/' + package_name + '/launch', ['launch/motor_duty_cycle_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rospi',
    maintainer_email='rospi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'motor_control_rpm = dc_motor_control.motor_control_rpm:main',
        'motor_duty_cycle = dc_motor_control.motor_duty_cycle:main',
        'motor_pid_exe = dc_motor_control.motor_pid:main',
        ],
    },
)
