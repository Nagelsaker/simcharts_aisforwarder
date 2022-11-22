from setuptools import setup

package_name = 'simcharts_aisforwarder'
packages = ['simcharts_aisforwarder', 'simcharts_aisforwarder/ais_msg', 'simcharts_aisforwarder/ais_msg_parser', 'simcharts_aisforwarder/api_readers', 'simcharts_aisforwarder/api_readers/norway_barents_watch', 'simcharts_aisforwarder/utils']

setup(
    name=package_name,
    version='0.0.1',
    packages=packages,
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
            'ais_publisher = simcharts_aisforwarder.ais_publisher:main',
            'ais_subscriber = simcharts_aisforwarder.ais_subscriber:main',
        ],
    },
)

