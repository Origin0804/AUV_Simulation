from setuptools import find_packages, setup

package_name = 'env_pkg'

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
    description='Environment simulation package for AUV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drag_node = env_pkg.drag_node:main',
            'props_node = env_pkg.props_node:main',
        ],
    },
)
