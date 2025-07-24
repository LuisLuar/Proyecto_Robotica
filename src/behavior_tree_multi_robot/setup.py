from setuptools import setup

package_name = 'behavior_tree_multi_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.tree_nodes'],  # Lista explícita de paquetes
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'py-trees', 'py-trees-ros'],
    zip_safe=True,
    maintainer='raynel',
    maintainer_email='luchitov2001@gmail.com',
    description='Control de múltiples robots con Behavior Trees',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bt_main = behavior_tree_multi_robot.bt_main:main',
            'gui = behavior_tree_multi_robot.simulador_gui:main'
        ],
    },
)