
# ====================
# Imports to add the launch file in the share directory
import os
from glob import glob
# ====================
from setuptools import find_packages, setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # --- Rule to copy all .py files inside the launch folder over the share directory ---
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Boris',
    maintainer_email='borisgangue@gmail.com',
    description='Publisher / Subscriber Nodes test using rclpy',
    license='Apache Licence 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "PubNode = py_pubsub.pub:main",
            "SubNode = py_pubsub.sub:main",
        ],
    },
)
