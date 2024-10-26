from setuptools import find_packages, setup

package_name = 'test_action'

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
    maintainer='vkj',
    maintainer_email='vijay.jadon@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "action_server_exe=test_action.action_server:main",
            "action_client_exe=test_action.action_client:main",
            "cruise_server_exe=test_action.cruise_server:main",
            "cruise_client_exe=test_action.cruise_client:main",
            "fibonacci_server_exe=test_action.fibonacci_server:main",
            "fibonacci_client_exe=test_action.fibonacci_client:main",
        ],
    },
)
