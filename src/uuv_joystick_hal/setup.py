from setuptools import find_packages, setup

package_name = 'uuv_joystick_hal'

setup(
    name=package_name,
    version='1.0.0',
    #packages=find_packages(exclude=['test']),
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yxz',
    description='Joystick hardware abstraction layer for UUV control',
    license='TODO: License declaration',
    
    entry_points={
        'console_scripts': [
            'joystick_hal = uuv_joystick_hal.input_node:main',
        ],
    },
)
