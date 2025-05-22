from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yahboom_camera_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools', 'tf2_ros', 'geometry_msgs'],
    zip_safe=True,
    maintainer='lolade',
    maintainer_email='anjuwonololade@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_controller = tf2_servo_controller.tf2_servo_node:main',
        ],
    },
)