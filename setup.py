from setuptools import find_packages, setup
from glob import glob

package_name = 'atlas_juggling'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/rviz',   glob('rviz/*')),
        ('share/' + package_name + '/urdf',   glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='Atlas Juggling',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'balldemo          = code.balldemo:main',
            'pirouetteandwave  = code.pirouetteandwave:main',
            'bend_knees        = code.bend_knees:main',
            'popasquat         = code.popasquat:main',
            'balls             = code.balls:main',
            'juggle            = code.juggle_demo:main',
        ],
    },
)
