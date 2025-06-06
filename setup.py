from setuptools import find_packages, setup

package_name = 'ros2_gopro_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'open-gopro', 'opencv-python', 'asyncio'],
    zip_safe=True,
    maintainer='fred',
    maintainer_email='nino.louman@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gopro_node = ros2_gopro_driver.gopro_node:main', 
        ],
    },
)
