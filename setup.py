from setuptools import find_packages, setup
from glob import glob

package_name = 'karate'

# Files copied from src -> install (share/karate/...).
otherfiles = [
    ('share/' + package_name + '/launch', glob('launch/*')),
    ('share/' + package_name + '/urdf',   glob('urdf/*')),
    ('share/' + package_name + '/meshes', glob('meshes/*')),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]+otherfiles,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='ME133a KarateBot',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'swing = karate.swing:main',
        ],
    },
)
