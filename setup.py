from setuptools import find_packages, setup

package_name = 'odrive_can'

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
    maintainer='ddon',
    maintainer_email='caoquangddon5@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_sender_node = odrive_can.keyboard_sender_node:main',
            'odrive_sender_node = odrive_can.odrive_sender:main',
            'test_sender_node = odrive_can.test_sender:main',
        ],
    },
)
