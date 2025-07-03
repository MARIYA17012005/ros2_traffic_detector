from setuptools import setup

package_name = 'traffic_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ponnu',
    maintainer_email='ponnu@example.com',
    description='Traffic light and sign detection using YOLOv8 in ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_traffic = traffic_detector.detector_node:main',
            'publish_test_image = traffic_detector.publish_test_image:main',
        ],
    },
)
