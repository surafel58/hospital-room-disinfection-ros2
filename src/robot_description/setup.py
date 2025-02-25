from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # URDF files
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        
        # Launch files 
        ('share/' + package_name + '/launch', glob('launch/*')),
        
        # RViz files
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        
        # Gazebo files
        ('share/' + package_name + '/gazebo', glob('gazebo/*')),
        
        # Mesh files
        ('share/' + package_name + '/meshes', glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='surafel',
    maintainer_email='surafelsenayehu58@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
