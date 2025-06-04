from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo para sensores de ultrasonidos (medidas)
        Node(
            package='mi_paquete',
            executable='ultrasonidos',
            name='ultrasonidos_node',
            output='screen',
            emulate_tty=True,
        ),
        
        # Nodo controlador - sigue paredes
        Node(
            package='mi_paquete',
            executable='controlador',
            name='controlador_node',
            output='screen',
            parameters=[
                {'target_distance': 0.3},
                {'max_correction': 0.5},
                {'kp': 1.0},
                {'ki': 0.0},
                {'kd': 0.0}
            ],
            emulate_tty=True,
        ),
        
        # Nodo para accionamiento de motores
        Node(
            package='mi_paquete',
            executable='motores',
            name='motores_node',
            output='screen',
            emulate_tty=True,
        ),
    ]) 