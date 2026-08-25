from setuptools import find_packages, setup

package_name = 'perception_node'

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
    maintainer='prasad',
    maintainer_email='prasadghumare7189@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'occupancy_mapper = perception_node.occupancy_mapper:main',
            'px4_odom_tf = perception_node.px4_odom_tf:main',
            'detector = perception_node.detector:main',
            'target_follow = perception_node.target_follow:main',
        ],
    },
)
