from setuptools import find_packages, setup

package_name = 'mobile_control'

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
    maintainer='uos',
    maintainer_email='uos@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mobile_robot = mobile_control.mobile_robot:main",
            "keyboard_controller = mobile_control.keyboard_control:main",
        ],
    },
)
