from setuptools import find_packages, setup

package_name = 'swarm_communication'

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
    description='Low-bandwidth heartbeat and status sharing across the swarm.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'heartbeat_node = swarm_communication.heartbeat_node:main',
        ],
    },
)
