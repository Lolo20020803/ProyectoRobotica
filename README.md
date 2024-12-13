# Control de Robot en Webots con Joystick Analógico usando ROS 2

## Descripción
Este proyecto permite controlar un robot simulado en Webots utilizando un joystick analógico, integrado con ROS 2 para la comunicación entre el simulador y los comandos de entrada del controlador. El sistema está diseñado para recibir las señales del joystick, procesarlas, y enviar comandos de movimiento al robot en Webots.

## Requisitos

### Hardware
- Joystick analógico compatible
- Ordenador con capacidad de ejecutar Webots y Ros 2 

### Software
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Webots R2023a](https://cyberbotics.com)
- Paquete `pygames` de Python para manejar entradas del joystick
- Python 3.8 o superior

## Instalación

### 1. Clonar el repositorio
```bash
cd ~/ros2_ws/src
git clone <URL_DEL_REPOSITORIO>
cd ~/ros2_ws
colcon build
```

### 2. Instalar dependencias
```bash
rosdep install --from-paths src --ignore-src -r -y
```
## Ejecución



### 1. Ejecutar launcher de Ros2
```bash
ros2 launch message_controller robot_launch.py
```

### 2. Verificar funcionamiento
Usa el joystick para mover el robot en Webots. Asegúrate de que el robot responde correctamente a los comandos de entrada.

## Estructura del Proyecto
```
/<nombre_proyecto>/
├── launch/
│   ├── jtobot_launch.py      # Archivo de lanzamiento para el projecto
├── message_controller/
│   ├── __init__.py             # Archivo que se modifica para ejecutar todo (no borrar)
│   ├── message_controller_Joystick.py             # Nodo que traduce comandos al topic
│   ├── my_robot_driver.py             # Nodo que traduce comandos del topic al robot
├── resources/
│   ├── message_controller       
│   ├── my_robot.urdf            # Configuración del robot
├── worlds/
│   ├── my_world.wbt            # Configuración del mundo de webots
├── package.xml                      # Información del paquete
├── setup.cfg                      
├── setup.py
├── .gitignore
└── README.md                        # Archivo actual
```

## Personalización
- Ajusta los valores de sensibilidad del joystick en `config/joystick_params.yaml` según las especificaciones del hardware.
- Modifica los parámetros del robot en `config/robot_params.yaml` para adaptarlos al modelo de Webots.

## Contribuciones
Las contribuciones son bienvenidas. Por favor, abre un *pull request* o un *issue* si encuentras problemas o tienes sugerencias para mejorar el proyecto.

## Licencia
Este proyecto está bajo la Licencia MIT. Consulta el archivo `LICENSE` para más detalles.

## Créditos
Este proyecto fue desarrollado por [Alex].