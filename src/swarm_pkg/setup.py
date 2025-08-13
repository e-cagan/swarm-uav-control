from setuptools import find_packages, setup

package_name = 'swarm_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cagan',
    maintainer_email='emincaganapaydin@gmail.com',
    description='Sürü iha kontrol etme paketi.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swarm_control_node = swarm_pkg.swarm_control_node:main'
        ],
    },
)
