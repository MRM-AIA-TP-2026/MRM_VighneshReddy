from setuptools import setup

package_name = 'robot_position'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy', 'nav_msgs'],
    data_files=[
        ('share/ament_index/resource_index/ament_package', ['package.xml']),
    ],
    zip_safe=True,
    maintainer='vighneshreddy',  
    maintainer_email='vighneshreddysatti@gmail.com',  
    description='A package to listen to odometry data and log the robot\'s position and orientation.',
    long_description='This package subscribes to the /odom topic and logs the robot\'s position and orientation in the console.',
    long_description_content_type='text/markdown',
    author='vighneshreddy',  
    author_email='vighneshreddysatti@gmail.com',  
    license='TODO: License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_position = robot_position.get_position:main',  
        ],
    },
)

