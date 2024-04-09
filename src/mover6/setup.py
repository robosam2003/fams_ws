from setuptools import find_packages, setup

package_name = 'mover6'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add the moverjoint and pcanbus files
        ('lib/' + package_name, [package_name + '/MoverJoint.py']),
        ('lib/' + package_name, [package_name + '/PCanBus.py']),   
        ('lib/' + package_name, [package_name + '/Controller_ui.py']),
  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robosam',
    maintainer_email='robosam2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mover6 = mover6.mover6:main',
            'mover6_control_gui = mover6.controllerNode:main',
        ],
    },
)
