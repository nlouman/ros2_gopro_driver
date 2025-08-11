from setuptools import find_packages, setup
import os
import sys

# 1. Prepend our local OpenGoPro path so "import open_gopro" points at external/OpenGoPro/python/
this_dir = os.path.dirname(__file__)
opengopro_path = os.path.join(this_dir, 'external', 'OpenGoPro', 'python')
sys.path.insert(0, opengopro_path)

package_name = 'ros2_gopro_driver'

setup(
    name=package_name,
    version='0.1.0',
    # find_packages() will pick up ros2_gopro_driver/ as well as any other submodules under that folder
    packages=find_packages(exclude=['test']),  
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), ['launch/gopro_launch.py']),
        (os.path.join('share', package_name, 'config'), ['config/gopro_config.yaml']),
    ],
    # We no longer list "open-gopro" here, because we're using the submodule checkout
    install_requires=[
        'setuptools',
        'opencv-python',    # if you need cv2
    ],
    zip_safe=True,
    maintainer='nlouman',
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
