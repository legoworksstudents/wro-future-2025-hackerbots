#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from buildhat import DistanceSensor, Motor
import time

class WallFollowerNode(Node):
    """
    Nodo seguidor de pared para un robot móvil.
    
    Este nodo integra la funcionalidad de los sensores ultrasónicos y los motores
    para implementar un algoritmo de seguimiento de pared. El robot seguirá
    una pared a su derecha manteniendo una distancia objetivo.
    """
    def __init__(self):
        """
        Inicializa el nodo seguidor de pared con sensores y motores.
        
        Configura los parámetros, inicializa hardware (sensores y motores),
        crea publicadores para datos de depuración y establece un temporizador
        para el bucle de control.
        """
        super().__init__('wall_follower_node')
        
        # Declaración de parámetros con valores predeterminados
        self.declare_parameter('velocidad_motor', 60.0)  # Velocidad base del motor trasero
        self.declare_parameter('target_distance', 0.2)   # Distancia objetivo a la pared (metros)
        self.declare_parameter('Kp', 45.0)               # Ganancia del controlador proporcional
        self.declare_parameter('Ki', 5.0)                # Ganancia del controlador integral
        self.declare_parameter('Kd', 10.0)               # Ganancia del controlador derivativo
        self.declare_parameter('max_integral', 30.0)     # Valor máximo para evitar saturación integral
        self.declare_parameter('wall_follower_active', True)

        # Obtener valores de los parámetros
        self.velocidad_motor = self.get_parameter('velocidad_motor').value
        self.target_distance = self.get_parameter('target_distance').value
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.max_integral = self.get_parameter('max_integral').value
        self.wall_follower_active = self.get_parameter('wall_follower_active').value

        # Variables para el controlador PID
        self.integral = 0.0          # Acumulación de errores (término integral)
        self.prev_error = 0.0        # Error anterior (para término derivativo)
        self.last_time = self.get_clock().now()  # Tiempo del último cálculo
        self.correction = 0.0
        
        # Inicializar sensores ultrasónicos
        # Puerto C: sensor izquierdo, Puerto D: sensor derecho
        self.left_sensor = DistanceSensor('C', 200)  # Sensor izquierdo
        self.right_sensor = DistanceSensor('D', 200)  # Sensor derecho (utilizado para seguir la pared)
        
        # Inicializar motores
        # Puerto A: motor trasero (propulsión), Puerto B: motor delantero (dirección)
        self.atras_motor = Motor("A")     # Motor trasero para la propulsión
        self.adelante_motor = Motor("B")  # Motor delantero para la dirección
        
        # Crear publicador para datos de sensores (para depuración)
        self.distance_publisher = self.create_publisher(
            Float32MultiArray, 'wall_follower/distances', 10)
        
        # Crear publicador para el valor de corrección (para depuración)
        self.correction_publisher = self.create_publisher(
            Float32, 'wall_follower/correction', 10)
            
        # Nuevos publicadores para velocidad y dirección
        self.speed_publisher = self.create_publisher(
            Float32, 'wall_follower/motor_speed', 10)
        self.steering_publisher = self.create_publisher(
            Float32, 'wall_follower/steering_angle', 10)
        

        self.subscriptioncamera= self.create_subscription(
            Float32,
            'cubos',
            self.guardar_cubos,
            10)

        # Temporizador para el bucle de control (10 Hz = cada 0.1 segundos)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # self.get_logger().info('Nodo Seguidor de Pared inicializado')
        # self.get_logger().info(f'Parámetros: velocidad_motor={self.velocidad_motor}, '
        #                        f'distancia_objetivo={self.target_distance}, '
        #                        f'Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}')
    

    def guardar_cubos(self, msg):
        self.get_logger().info(f'Recibido: {msg.data}')

        self.cubos=msg.data

        kpc = 1
        self.correction = self.cubos[0] * kpc
        if self.cubos[1]=="verde":
            self.correction * -1
        else:
            self.correction * 1


        if not self.cubos:
            self.get_logger().info('No hay cubos')
        else:
            self.get_logger().info(f'Cubos: {self.cubos}')

    def control_loop(self):
        """
        Bucle principal de control para el seguimiento de pared.
        
        Este método se ejecuta periódicamente (10 veces por segundo) y realiza:
        1. Lectura de los sensores ultrasónicos
        2. Cálculo del error y la corrección necesaria mediante control PID
        3. Control de los motores para mantener la distancia objetivo a la pared
        """
        # Leer valores de los sensores
        try:
            # Convertir de milímetros a metros
            left_distance = float(self.left_sensor.get_distance()) / 1000  # Distancia izquierda en metros
        except Exception as e:
            self.get_logger().error(f'Error al leer sensor izquierdo: {e}')
            left_distance = float('inf')  # Valor infinito en caso de error
            
        try:
            # Convertir de milímetros a metros
            right_distance = float(self.right_sensor.get_distance()) / 1000  # Distancia derecha en metros
        except Exception as e:
            self.get_logger().error(f'Error al leer sensor derecho: {e}')
            right_distance = float('inf')  # Valor infinito en caso de error
        
        # Publicar datos de sensores para depuración
        distance_msg = Float32MultiArray()
        distance_msg.data = [left_distance, right_distance]
        self.distance_publisher.publish(distance_msg)
        
        # Calcular tiempo transcurrido desde la última iteración
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convertir a segundos
        self.last_time = current_time
        
        # Evitar divisiones por cero o valores dt muy pequeños
        if dt < 0.001:
            dt = 0.001
            
        # Calcular error (positivo = muy lejos, negativo = muy cerca)
        error = left_distance - right_distance
        
        # Calcular término proporcional (P)
        p_term = self.Kp * error
        
        # Calcular término integral (I)
        self.integral += error * dt
        # Limitar el término integral para evitar "wind-up"
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        i_term = self.Ki * self.integral
        
        # Calcular término derivativo (D)
        derivative = (error - self.prev_error) / dt
        d_term = self.Kd * derivative
        self.prev_error = error  # Guardar error actual para la próxima iteración
        
        # Combinar los tres términos para obtener la corrección total
        correction = p_term + i_term + d_term
        
        # Limitar la corrección a valores razonables (±45 grados)
        correction = max(-80.0, min(60.0, correction))
        
        # Publicar valor de corrección para depuración
        correction_msg = Float32()
        correction_msg.data = correction
        self.correction_publisher.publish(correction_msg)
        
        # Controlar motores directamente
        # El motor delantero controla la dirección: 90° es recto, <90° gira izquierda, >90° gira derecha

        if self.wall_follower_active:
            steering_angle = 0 + correction + self.correction
        else:
            steering_angle = 0 + self.correction
        
        self.adelante_motor.run_to_position(int(steering_angle))
        
        # El motor trasero proporciona la potencia de avance constante
        self.atras_motor.start(speed=int(self.velocidad_motor))
        
        # Publicar velocidad y ángulo de dirección
        speed_msg = Float32()
        speed_msg.data = float(self.velocidad_motor)
        self.speed_publisher.publish(speed_msg)
        
        steering_msg = Float32()
        steering_msg.data = steering_angle
        self.steering_publisher.publish(steering_msg)
        
        # Registrar información para depuración
        self.get_logger().info(
            f'Izquierda: {left_distance:.2f}m, Derecha: {right_distance:.2f}m, '
            f'Corrección: {correction:.2f} (P:{p_term:.2f}, I:{i_term:.2f}, D:{d_term:.2f})'
        )
    
    def destroy_node(self):
        """
        Limpia y detiene todos los motores y sensores al finalizar el nodo.
        Esto es importante para evitar que los motores sigan funcionando
        después de cerrar el programa.
        """
        # Apagar todos los motores y sensores
        self.atras_motor.stop()
        self.adelante_motor.stop()
        # self.left_sensor.stop()
        # self.right_sensor.stop()
        super().destroy_node()

def main(args=None):
    """
    Función principal que inicializa el nodo y lo mantiene en ejecución.
    
    Esta función:
    1. Inicializa la biblioteca rclpy
    2. Crea una instancia del nodo
    3. Mantiene el nodo en ejecución hasta que se interrumpa (Ctrl+C)
    4. Limpia recursos al finalizar
    """
    rclpy.init(args=args)
    node = WallFollowerNode()
    try:
        # Mantener el nodo en ejecución
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Manejar Ctrl+C
        pass
    finally:
        # Asegurar que se limpien los recursos
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 