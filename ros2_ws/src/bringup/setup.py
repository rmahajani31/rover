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
            'launch/fast_lio2_nav2.launch.py',
            'launch/fastlio_shadow.launch.py',
            'launch/jetson.launch.py',
            'launch/lio_ekf_deskew_costmap_projection.launch.py',
            'launch/scan_to_scan_icp.launch.py',
            'launch/mapping.launch.py',
            'launch/costmap_projection.launch.py',
            'launch/pointcloud_preprocess_fast_lio2_nav2.launch.py',
            'launch/pi_fast_lio2_costmap_projection_nav2.launch.py',
            'launch/pi_fast_lio2_nav2.launch.py',
            'launch/pi_nav2_livox.launch.py',
            'launch/scan_to_map_costmap_projection.launch.py',
            'launch/scan_to_map_deskew_costmap_projection.launch.py',
            'launch/scan_to_map_primary_odom.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/mid360_fastlio2.yaml',
            'config/nav2_params_fast_lio2_costmap_projection.yaml',
            'config/nav2_params_fast_lio2_nav2.yaml',
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
