from setuptools import find_packages, setup

package_name = 'emergency_camera'

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
    maintainer='juyeong',
    maintainer_email='ju0gkorea@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detector = emergency_camera.yolo_detector:main',
            'control_tower = emergency_camera.control_tower:main',
            'demo_robot_client = emergency_camera.demo_robot_client:main',
        ],
    },
)
