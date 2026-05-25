from setuptools import find_packages, setup

package_name = 'bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/rover.launch.py',
            'launch/jetson_launch.py',
            'launch/pi_nav2_livox.launch.py',
        ]),
        ('share/' + package_name + '/urdf', ['urdf/rover.urdf']),
        ('share/' + package_name + '/config', [
            'config/slam_toolbox_async.yaml',
            'config/nav2_params_livox.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rmahajani',
    maintainer_email='rmahajani@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
