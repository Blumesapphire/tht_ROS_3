from setuptools import find_packages, setup

package_name = 'lamp_control'

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
    maintainer='blumesapphire',
    maintainer_email='blumesapphire@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'processor_node = lamp_control.process_node:main',
        'sensor_node = lamp_control.sensor_node:main',
        'lamp_service = lamp_control.lamp_service:main',
        'lamp_action = lamp_control.lamp_action:main',
    ],
    },
)
