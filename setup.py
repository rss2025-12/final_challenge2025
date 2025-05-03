from setuptools import find_packages, setup
import os, glob

package_name = 'heist'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/heist/launch/sim', glob.glob(os.path.join('launch', 'sim', '*launch.*'))),
        ('share/heist/launch/real', glob.glob(os.path.join('launch', 'real', '*launch.*'))),
        (os.path.join('share', package_name, 'config', 'sim'), glob.glob('config/sim/*.yaml')),
        (os.path.join('share', package_name, 'config', 'real'), glob.glob('config/real/*.yaml')),
        ('lib/'+package_name+"/computer_vision", glob.glob(os.path.join('heist/computer_vision', '*.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'light_detector = heist.light_detector:main',
            'detection_node = heist.model.detection_node:main',
            'basement_point_publisher = heist.basement_point_publisher:main',
            'obstacle_publisher = heist.obstacle_publisher:main',
            'state_machine = heist.state_machine:main',
        ],
    },
)
