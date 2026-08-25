from setuptools import find_packages, setup

package_name = 'mission_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={'mission_interface': ['mission_plan.schema.json']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python-dotenv', 'huggingface_hub'],
    zip_safe=True,
    maintainer='prasad',
    maintainer_email='prasadghumare7189@gmail.com',
    description='Mission entry point: prompt/JSON -> validate -> /mission_intent.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'prompt_gateway = mission_interface.prompt_gateway:main',
            'mission_supervisor = mission_interface.mission_supervisor:main',
        ],
    },
)
