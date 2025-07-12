from setuptools import setup
import sys
sys.executable = "/home/ponnu/yolovenv/bin/python3"

package_name = 'traffic_sign_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/traffic_sign_detector']),
        ('share/' + package_name + '/models', ['models/best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ponnu',
    maintainer_email='ponnu@example.com',
    description='Traffic sign detection using YOLOv8 in ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_sign_infer = traffic_sign_detector.traffic_sign_infer:main',
            'image_publisher_node = traffic_sign_detector.image_publisher_node:main',
        ],
    },
)
