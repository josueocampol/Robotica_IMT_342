### Descripción general
El proyecto AGV Teleoperado con ROS 2 y ESP8266 es un paquete ROS 2 diseñado para simular y configurar los datos de avance de un robot móvil en un entorno Gazebo. El paquete incluye modelos URDF y SDF del robot, un archivo de mundo y archivos de lanzamiento para iniciar la simulación con Gazebo y RViz.

### image 
<center><img src="https://res.cloudinary.com/diekemzs9/image/upload/v1743203741/Screenshot_from_2025-03-29_00-59-50_ohbt5j.png"/></center>


## Features
Este robot esta simulado en un entorno pequeño utilizando Gazebo.
Visualice el estado y el entorno del robot en RViz.
Controlar robot con rqt
Incluye modelos URDF y SDF para la descripción y simulación de robots.

## Visión G eneral

el modelo Mobi para la descripción y simulación de robots.

## Prerequisites

ROS 2 Humble Hawksbill o posterior
Mirador 11
RViz

## Configuración

1. **Clonar el repositorio:**

   ```bash
   cd basic_mobile_robot
   ```

2. **Construya el paquete:**

   ```bash
   colcon build --packages-select basic_mobile_robot
   ```

3. **Fuente del archivo de instalación:**

   ```bash
   source install/setup.bash
   ```

## Para Usar

1. **Iniciar la simulación:**

   Para iniciar la simulación con el mundo de la pequeña ciudad y el modelo de robot móvil, ejecute:

   ```bash
   ros2 launch basic_mobile_robot basic_mobile_bot_v2.launch.py
   ```

2. **Visualize in RViz:**

RViz se iniciará automáticamente si está habilitado en el archivo de inicio. Puede ver el estado y el entorno del robot.


## Estructura de Archivos

"launch"/:Contiene archivos de lanzamiento para iniciar la simulación.
models/:Contiene modelos URDF y SDF del robot.
worlds/:Contiene el archivo del mundo de la pequeña ciudad de Gazebo.
rviz/:Contiene archivos de configuración de RViz.


## Author

Josue Ocampo  

