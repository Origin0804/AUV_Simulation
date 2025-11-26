from setuptools import find_packages, setup

package_name = 'AUV_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AUV Team',
    maintainer_email='your@email.com',
    description='AUV simulation package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position_node = AUV_pkg.position_node:main',
            'motor_node = AUV_pkg.motor_node:main',
            'camera_node = AUV_pkg.camera_node:main',
            'main_node = AUV_pkg.main_node:main',
            'dvl_node = AUV_pkg.dvl_node:main',
            'imu_node = AUV_pkg.imu_node:main',
        ],
    },
)
