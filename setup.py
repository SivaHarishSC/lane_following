import os
from glob import glob
from setuptools import setup

package_name = 'jetracer2ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='af',
    maintainer_email='siv2871s@hs-coburg.de',
    maintainer_email='sri6668s@hs-coburg.de',
     maintainer_email='lab1526s@hs-coburg.de',
    description='Package that implements jetracer Apex detection and movement request in ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jetracer_ros_node = jetracer2ros.jetracer_ros_node:main'
        ],
    },
)
