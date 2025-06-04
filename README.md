# WRO Future Engineers - Vehículo Autónomo

## Equipo
Equipo: Hackerbots LegoWorks
Integrantes: 3
País/Región: España/Navarra

## Descripción del Proyecto
Este repositorio contiene el código y la documentación de nuestro vehículo autónomo para la competición WRO Future Engineers. El objetivo del proyecto es desarrollar un vehículo capaz de navegar autónomamente en un circuito siguiendo señales de tráfico (pilares rojos y verdes) y completando tres vueltas en el menor tiempo posible.

## Estructura del Vehículo
Nuestro vehículo utiliza un diseño de tracción trasera con un motor brushless para propulsión y un motor grande de LEGO para la dirección. Los componentes principales incluyen:

- Controlador: Raspberry Pi (para procesamiento de visión) y motor controlador BuildHat
- Arduino Portenta h7 para controlar motores y sensores. 
- Sensores:
  - Cámara USB para detección de objetos de colores
  - Sensores ultrasónicos para detección de distancia de LEGO
- Motor brushless para propulsión
- Motor grande de LEGO Spike para controlar dirección
- ESC del motor brushless

## Estructura del Repositorio
- `/ros2/src/mi_paquete`: Paquete ROS2 principal
  - `/mi_paquete`: Código Python de los nodos
    - `wall_follower.py`: Control principal del vehículo
    - `camera.py`: Procesamiento de visión por computadora
    - `ultrasonidos.py`: Gestión de sensores de distancia
    - `motores.py`: Control de motores
    - `controlador.py`: Algoritmos de control PID
  - `/launch`: Archivos de lanzamiento para ROS2
  - `/resource`: Recursos adicionales

## Algoritmo de Control
Nuestro vehículo utiliza una combinación de algoritmos para la navegación autónoma:

1. **Seguimiento de Pared**: Utilizamos un controlador PID para mantener una distancia constante respecto a las paredes del circuito.
2. **Detección de Objetos**: La cámara detecta los pilares de colores (rojo y verde) y determina su posición relativa al centro de la camara en forma de porcentaje.
3. **Toma de Decisiones**: Basado en la información de los sensores, el algoritmo decide la velocidad y dirección óptimas.

## Tecnologías Utilizadas
- ROS2 (Robot Operating System 2)
- Python
- OpenCV para procesamiento de imágenes
- Biblioteca BuildHat para control de motores
- Controlador PID para estabilidad

## Instalación y Ejecución

### Requisitos Previos
- Ubuntu 20.04 o superior
- ROS2 Foxy o superior
- Python 3.8+
- OpenCV 4.2+
- Bibliotecas BuildHat

### Hardware y Software:

Para realizar medidas y una implementación rápida hemos usado piezas LEGO asi como el buildhat para usar los motores y sensores del Spike con Raspberry. 

Todo ha sido programado con Python y bash. Todo el código corre en un contenedor de Ubuntu para evitar problemas de compatibilidad. 

Utilizamos una Raspberry para poder utilizar ROS2. Este nos permite crear rutinas y procesos para programar por nodos y hacer una programación por partes.

Al principio para propulsión usamos un motor grande de LEGO, pero el buildhat no da la corriente necesaria para tener un empuje decente. Cambiamos a un motor brushless de coches RC de 20.000 KV para no tener problemas de potencia. Este lo controlamos con una portenta H7 y su respectivo ESC. 

Utilizamos una portenta H7 por si teniamos problemas con los sensores ultrasonicos de LEGO tener un aparato fiable para mandar datos via serial a la Raspberry ya que esta trabaha a 480 Mhz. 

El coche utiliza dos baterias. Una de 7,4V para la raspberry y el buildhat y otra de 12,4V para el motor brushless. 

Para los soportes de los componentes que no son de LEGO hemos hecho piezas en 3D asi como la correa de transmisión del motor. 

Para el proximo año queremos implementar la navegación con un Lidar 2D. 

A este proyecto le hemos dedicado 1h a la semana durante 2 meses. 

### Ejecución
```bash
# Iniciar el sistema completo
ros2 launch mi_paquete wall_follower.launch.py

# Ejecutar solo el nodo de cámara para pruebas
ros2 run mi_paquete camera.py
```

## Desafíos y Soluciones
- **Desafío 1**: Seguir correctamene el recorrido.
  - *Solución*: Dos sensores ultrasonicos que comprueban la distancia a la izquierda y derecha del robot. 
  
- **Desafío 2**: Detectar y evitar pilares.
  - *Solución*: Añadimos un parámetro a nuestro PID para que corrija en función de la distancia del bloque al centro de la imagen. 


## Licencia
Este proyecto es libre.

## Agradecimientos
- Agradecemos a nuestros mentores y patrocinadores por su apoyo.
- Reconocimiento especial a [mencionar si aplica]. 
