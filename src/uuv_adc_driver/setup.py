from setuptools import find_packages, setup

package_name = 'uuv_adc_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/adc_driver.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Berk Yilmaz',
    maintainer_email='berkyilmaz2005@gmail.com',
    description='ADC Driver for BlueRobotics Flightcontroller',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'adc_publisher = uuv_adc_driver.adc_driver:main',
        ],
    },
)
