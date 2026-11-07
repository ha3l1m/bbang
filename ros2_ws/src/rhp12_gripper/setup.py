from glob import glob
from setuptools import setup

package_name = 'rhp12_gripper'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stu',
    maintainer_email='smoodworks@gmail.com',
    description='RH-P12-RN-A gripper model and control bridge for Doosan E0509',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gripper_node = rhp12_gripper.gripper_node:main',
        ],
    },
)
