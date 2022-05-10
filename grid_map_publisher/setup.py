import os
from glob import glob
from setuptools import setup

package_name = 'grid_map_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'example'), glob('example/*.npz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='sadergachev@edu.hse.ru',
    description='Package with simple grid_map publisher for 2.5D navigation debug',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grid_map_publisher = grid_map_publisher.grid_map_publisher:main',
        ],
    },
)
