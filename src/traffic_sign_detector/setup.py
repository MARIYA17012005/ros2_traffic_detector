from setuptools import setup

package_name = 'traffic_sign_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ponnu',
    maintainer_email='ponnu@todo.todo',
    description='Traffic sign detection package using ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'image_publisher_node = traffic_sign_detector.image_publisher_node:main',
        'traffic_sign_node = traffic_sign_detector.traffic_sign_node:main',
        'classifier_node = traffic_sign_detector.classifier_node:main',
    ],
},
)

