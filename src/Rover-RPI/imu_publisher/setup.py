from setuptools import find_packages, setup

package_name = 'imu_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harsh',
    maintainer_email='harshvardhan.shah43@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_serial_node = imu_publisher.imu_basic_publisher:main',
	        'bno055_imu_ekf = imu_publisher.imu_bno055_ekf_publisher:main',
            'bno055-imu_odom = imu_publisher.imu_bno055_odom_publisher:main',
            'bno085_imu_ekf_node = imu_publisher.imu_bno085_ekf_pub:main'
        ],
    },
)
