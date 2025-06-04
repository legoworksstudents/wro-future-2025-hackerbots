from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mi_paquete'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='legoworkspamplona@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mi_primer_nodo = mi_paquete.mi_primer_nodo:main',
            'contador = mi_paquete.contador_subscriber:main',
            'ultrasonidos = mi_paquete.ultrasonidos:main',
            'controlador = mi_paquete.controlador:main',
            'motores = mi_paquete.motores:main',
            'wall_follower = mi_paquete.wall_follower:main',
            'camera = mi_paquete.camera:main',
            
        ],
    },
)
