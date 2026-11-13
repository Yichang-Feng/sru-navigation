from setuptools import setup

package_name = 'go2w_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            ['launch/go2w_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiewang',
    maintainer_email='wjjy1123@163.com',
    description='ROS 2 control and state bridge for Unitree Go2W',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = go2w_bridge.cmd_vel_bridge:main',
            'wireless_controller = go2w_bridge.wireless_controller:main',
        ],
    },
)
