from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Crea y devuelve la descripción de lanzamiento con el nodo seguidor de pared.
    
    Este archivo de lanzamiento configura el nodo wall_follower con parámetros
    predefinidos para el seguimiento de pared.
    """
    
    # Nodo seguidor de pared que combina sensores ultrasónicos y motores
    wall_follower_node = Node(
        package='mi_paquete',                   # Nombre del paquete
        executable='wall_follower',             # Nombre del ejecutable
        name='wall_follower_node',              # Nombre del nodo en ejecución
        output='screen',                        # Mostrar salida en pantalla
        emulate_tty=True,                       # Emular terminal para colores
        parameters=[
            {
                'velocidad_motor': -100.0,        # Velocidad base del motor trasero
                'target_distance': 0.2,         # Distancia objetivo a la pared en metros
                'Kp': 1000.0,                     # Ganancia del controlador proporcional
                'Ki': 0.0,                      # Ganancia del controlador integral
                'Kd': 0.0,                     # Ganancia del controlador derivativo
                'max_integral': 30.0,           # Límite para evitar saturación integral
            }
        ]
    )
    
    # Crear la descripción de lanzamiento y añadir el nodo
    ld = LaunchDescription()
    ld.add_action(wall_follower_node)
    
    return ld 