from setuptools import find_packages, setup

package_name = 'helios_ii'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ben Nack',
    maintainer_email='blueoompaloompa72@gmail.com',
    description='Flight Controller for the HeliosII',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_action_server = helios_ii.servo_action_server:main'
        ],
    },
)
