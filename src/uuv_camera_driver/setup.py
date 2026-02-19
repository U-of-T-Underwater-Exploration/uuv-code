from setuptools import find_packages, setup

package_name = 'uuv_camera_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/uuv_camera_driver']),
        ('share/uuv_camera_driver', ['package.xml']),
        ('share/uuv_camera_driver/launch', ['launch/camera_driver_launch.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeff',
    maintainer_email='jeffshun123@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
