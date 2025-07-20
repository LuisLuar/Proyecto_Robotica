from setuptools import find_packages, setup

package_name = 'robot_movil'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/car.launch.py']),
        ('share/' + package_name + '/launch', ['launch/multi_ekf.launch.py']),
        ('share/' + package_name + '/launch', ['launch/multi_robot_display.launch.py']),
        ('share/' + package_name + '/launch', ['launch/multi_robot_path.launch.py']),
        ('share/' + package_name + '/launch', ['launch/principal.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raynel',
    maintainer_email='luchitov2001gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cuadrado = robot_movil.circle_reference:main',
            'trayectoria = robot_movil.trayectoria:main',
            'guardar = robot_movil.registrar_datos:main',
            'puentebt = robot_movil.puente_BT:main',
        ],
    },
)
