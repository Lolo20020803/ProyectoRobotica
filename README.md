# Control de Robot en Webots con Joystick Analógico usando ROS 2

## Descripción
Este proyecto permite controlar un robot simulado en Webots utilizando un joystick analógico, integrado con ROS 2 para la comunicación entre el simulador y los comandos de entrada del controlador. El sistema está diseñado para recibir las señales del joystick, procesarlas, y enviar comandos de movimiento al robot en Webots.

## Requisitos

### Hardware
- Joystick analógico compatible (Xbox, PlayStation, Nintendo Switch o Mando Generico)
- Ordenador con capacidad de ejecutar Webots y Ros 2 

### Software
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Webots R2023a](https://cyberbotics.com)
- Python 3.8 o superior

## Instalación

### 1. Clonar el repositorio
```bash
git clone https://github.com/Lolo20020803/ProyectoRobotica.git
cd ~/ProyectoRobotica
colcon build
```

### 2. Instalar dependencias

```bash
pip install pygame
```
## Ejecución


### 1 Hacer source del install
Comprueba los posibles archivos dentro de la carpeta `install`, ya que depende de tu terminal.

```bash
source /install/local_setup.*
```

### 2. Ejecutar launcher de Ros2
```bash
ros2 launch message_controller robot_launch.py
```


### 3. Verificar funcionamiento
Usa el joystick para mover el robot en Webots. Asegúrate de que el robot responde correctamente a los comandos de entrada.

## Estructura del Proyecto 
Esta estructura se completa al hacer el build del proyecto.
```
ProyectoRobotica
├── build
├── install
├── log
├── message_controller
│ ├── launch
│ │   └── robot_launch.py
│ ├── message_controller
│ │   ├── __init__.py
│ │   ├── message_controller copy.py
│ │   ├── message_controller.py
│ │   └── my_robot_driver.py
│ ├── resource
│ │   ├── message_controller
│ │   └── my_robot.urdf
│ ├── worlds
│ │   ├── my_world copy.wbt
│ │   └── my_world.wbt
│ ├── package.xml
│ ├── setup.cfg
│ ├── setup.py
└── README.md
```

## Contribuciones
Las contribuciones son bienvenidas. Por favor, abre un *pull request* o un *issue* si encuentras problemas o tienes sugerencias para mejorar el proyecto.

