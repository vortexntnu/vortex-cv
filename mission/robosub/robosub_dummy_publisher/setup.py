from glob import glob

from setuptools import find_packages, setup

package_name = 'robosub_dummy_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/robosub_dummy_publisher'],
        ),
        ('share/robosub_dummy_publisher', ['package.xml']),
        ('share/robosub_dummy_publisher/launch', glob('launch/*.launch.py')),
        ('share/robosub_dummy_publisher/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hinthujan',
    maintainer_email='Hinthu5@hotmail.no',
    description=(
        'Seeded dummy LandmarkArray publishers standing in for perception on '
        'each RoboSub course element (gate, slalom, torpedo board, bins).'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'robosub_dummy_publisher_node = '
            'robosub_dummy_publisher.robosub_dummy_publisher_node:main',
            'detections_markers_node = '
            'robosub_dummy_publisher.detections_markers_node:main',
        ],
    },
)
