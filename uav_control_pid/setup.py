from setuptools import setup

package_name = 'uav_control_pid'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/offboard_hover.launch.py',
            'launch/offboard_pid_goto.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/offboard_hover.yaml',
            'config/offboard_pid_goto.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uav-dev',
    maintainer_email='imhyeonwoo21@gmail.com',
    description='PX4 Offboard hover demo using /uav/odom as state input',
    license='proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_hover = uav_control_pid.offboard_hover:main',
            'offboard_pid_goto = uav_control_pid.offboard_pid_goto:main',
        ],
    },
)
