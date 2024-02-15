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
            "tst_node = mobile_control.my_first_node:main",
            "drw_crl = mobile_control.draw_circle:main"
        ],
    },
)