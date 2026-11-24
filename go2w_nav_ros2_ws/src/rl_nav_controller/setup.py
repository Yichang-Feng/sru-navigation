from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rl_nav_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'deployment_policies'), glob('deployment_policies/*.onnx')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'opencv-python',
        # 注意：不要在这里固定 onnxruntime。
        # Jetson 上需要的是 onnxruntime-gpu（见 setup_ros2_onnx.sh / start_local_sru.sh），
        # 而在 install_requires 里写 onnxruntime 会拉入 CPU 版并覆盖 GPU 版。
    ],
    zip_safe=True,
    maintainer='Fan Yang',
    maintainer_email='fanyang1@ethz.ch',
    description='Reinforcement Learning-based Navigation Controller for autonomous robot navigation using depth images and odometry (ONNX Runtime)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_nav_controller = rl_nav_controller.rl_nav_controller:main',
            'rl_nav_controller_adaptive = rl_nav_controller.rl_nav_controller_adaptive:main',
            'rl_nav_controller_stairs = rl_nav_controller.rl_nav_controller_stairs:main',
        ],
    },
)
