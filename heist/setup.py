from setuptools import find_packages, setup
import os, glob

package_name = 'shrinkray_heist'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/shrinkray_heist/launch', glob.glob(os.path.join('launch', '*launch.xml'))),
        ('lib/'+package_name+"/computer_vision", glob.glob(os.path.join('shrinkray_heist/computer_vision', '*.py')))
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
            'light_detector = shrinkray_heist.light_detector:main',
            'detection_node = shrinkray_heist.model.detection_node:main'
        ],
    },
)
