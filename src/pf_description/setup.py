import os
from glob import glob
from setuptools import setup

package_name = 'pf_description'

setup(
    name=package_name,
    version='1.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harsh Deshpande',
    maintainer_email='harshavardhan.deshpande@ipa.fraunhofer.de',
    description='The pf_description package',
    license='Apache2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)