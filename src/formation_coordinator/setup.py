from setuptools import find_packages, setup

package_name = 'formation_coordinator'

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
    description='Executes a MissionPlan phase by phase into per-drone DroneTasks.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'coordinator_node = formation_coordinator.coordinator_node:main',
            'drone_agent = formation_coordinator.drone_agent:main',
        ],
    },
)
