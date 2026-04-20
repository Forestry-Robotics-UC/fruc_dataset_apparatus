from setuptools import setup
from glob import glob
import os

package_name = 'apparatus_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # URDF files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        # Config files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # Meshes
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.STL'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.obj'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.mtl'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@example.com',
    description='CURT Mini robot URDF description',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
