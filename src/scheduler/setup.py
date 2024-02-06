from setuptools import find_packages, setup

package_name = 'scheduler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name + '/JobLog.csv']), # Add JSON file to the package
        ('lib/' + package_name, [package_name + '/message_converter.py']),     
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
            'scheduler = scheduler.scheduler:main',
        ],
    },
)
