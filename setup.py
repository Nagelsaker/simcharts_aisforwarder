from setuptools import setup

package_name = 'AIS_Forwarder'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simon J.N. Lexau',
    maintainer_email='simon.lexau@ntnu.no',
    description='Reads live AIS data from The Norwegian Coastal Administration "BarentsWatch" API, and publish the information as a ROS 2 topic',
    license='The MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
