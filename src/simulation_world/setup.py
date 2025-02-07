from setuptools import find_packages, setup

package_name = 'simulation_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', ['worlds/hospital_area.world']),
        ('share/' + package_name + '/worlds', ['worlds/hosp.world']),
        ('share/' + package_name + '/launch', ['launch/gazebo_sim.launch.py']),
        ('share/' + package_name + '/launch', ['launch/gazebo_sim.launch_h.py']),        
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
