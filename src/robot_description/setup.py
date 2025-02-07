from setuptools import find_packages, setup

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
        ('share/' + package_name + '/urdf', ['urdf/disinfection_robot.urdf.xacro']),
        ('share/' + package_name + '/urdf', ['urdf/ajrobot.trans']),
        ('share/' + package_name + '/urdf', ['urdf/materials.xacro']),
        ('share/' + package_name + '/urdf', ['urdf/ajrobot.xacro']),
        ('share/' + package_name + '/urdf', ['urdf/ajrobot.urdf']),
        
        # Launch files
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        ('share/' + package_name + '/launch', ['launch/gazebo.launch.py']),
        ('share/' + package_name + '/launch', ['launch/hosp.launch.py']),
        ('share/' + package_name + '/launch', ['launch/controller.launch.py']),
        ('share/' + package_name + '/launch', ['launch/controller.yaml']),
        
        # RViz files
        ('share/' + package_name + '/rviz', ['rviz/urdf_config.rviz']),
        
        # Gazebo files
        ('share/' + package_name + '/gazebo', ['gazebo/ajrobot_detailed_materials.gazebo']),
        ('share/' + package_name + '/gazebo', ['gazebo/ajrobot_detailed_physics.gazebo']),
        ('share/' + package_name + '/gazebo', ['gazebo/ajrobot_detailed_plugins.gazebo']),
        
        # Mesh files
        ('share/' + package_name + '/meshes', ['meshes/base_link.stl']),
        ('share/' + package_name + '/meshes', ['meshes/camera_1.stl']),
        ('share/' + package_name + '/meshes', ['meshes/casterb_1.stl']),
        ('share/' + package_name + '/meshes', ['meshes/casterf_1.stl']),
        ('share/' + package_name + '/meshes', ['meshes/lidar_1.stl']),
        ('share/' + package_name + '/meshes', ['meshes/lwheel_1.stl']),
        ('share/' + package_name + '/meshes', ['meshes/pir_1.stl']),
        ('share/' + package_name + '/meshes', ['meshes/rwheel_1.stl']),
        ('share/' + package_name + '/meshes', ['meshes/tube_frame_1.stl']),
        ('share/' + package_name + '/meshes', ['meshes/tubebl_1.stl']),
        ('share/' + package_name + '/meshes', ['meshes/tubebr_1.stl']),
        ('share/' + package_name + '/meshes', ['meshes/tubefl_1.stl']),
        ('share/' + package_name + '/meshes', ['meshes/tubefr_1.stl']),
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
