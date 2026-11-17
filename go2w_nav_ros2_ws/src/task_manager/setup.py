from setuptools import find_packages, setup

package_name = 'task_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml', 'README.md', 'mission-protocol-handoff.md',
            'navigation-integration-handoff.md',
        ]),
        ('share/' + package_name + '/launch', ['launch/task_manager.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Mission protocol executor using goal and planner status topics.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_manager_node = task_manager.task_manager_node:main',
        ],
    },
)
