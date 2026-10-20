from setuptools import setup

package_name = 'lidar_dynamic_map'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='STEMworks LiDAR Project',
    maintainer_email='user@example.com',
    description='Stationary LiDAR dynamic point-cloud mapping node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_mapping = lidar_dynamic_map.lidar_mapping:main',
        ],
    },
)
