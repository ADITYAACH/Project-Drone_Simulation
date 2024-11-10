from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'drone_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name, 'launch'),glob('launch/*')), 
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'models'), ['models/database.config']),
        (os.path.join('share', package_name, 'models', 'drone_again'), glob('models/drone_again/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'world'), glob('world/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('lib', package_name), glob('plugins/*.so')),
        (os.path.join('include', package_name), glob('include/' + package_name + '/*.h')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='achanti',
    maintainer_email='achanti@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'auto = drone_simulation.drone_control:main',
            	'teleop = drone_simulation.teleop_control:main',
            	'con = drone_simulation.control:main',
            	'teleop_joystick = drone_simulation.teleop_joystick:main',
            	'spawn_drone = drone_simulation.spawn_drone:main',
        ],
    },
)
