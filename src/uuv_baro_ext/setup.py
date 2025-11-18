from setuptools import find_packages, setup

package_name = 'uuv_baro_ext'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/external_barometer_launch.py']),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='frank',
    maintainer_email='frank.j.chen2007@gmail.com',
    description='External Barometer Data Publisher',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'uuv_baro_ext_publisher = uuv_baro_ext.baro_ext:main',
            ],
    },
)
