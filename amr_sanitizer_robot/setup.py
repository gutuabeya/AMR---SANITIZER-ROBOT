from setuptools import find_packages, setup
import os
package_name = 'amr_sanitizer_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/task3.launch.py']),
        (os.path.join('share', package_name, 'launch'), ['launch/task4.launch.py']),        
    ],
    install_requires=['setuptools',
    'rclpy',
    'nav_msgs',
    'tf2_ros',
    'numpy',
    'matplotlib',
    'scipy',
    'nav2_costmap_2d'],
    zip_safe=True,
    maintainer='gutuabeya',
    maintainer_email='gutuabeya@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laserscan = amr_sanitizer_robot.laserscan:main',
            'localize = amr_sanitizer_robot.localize:main',
            'sanitize = amr_sanitizer_robot.sanitize:main',
            'RouteManager = amr_sanitizer_robot.RouteManager:main',  
        ],
    },
)

