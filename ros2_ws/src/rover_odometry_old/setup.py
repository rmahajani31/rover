from setuptools import find_packages, setup

package_name = 'rover_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/rover_odometry.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rishabh Mahajani',
    maintainer_email='rmahajani31@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odometry = rover_odometry.odometry_node:main',
        ],
    },
)
