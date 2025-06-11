# Instalación de ROS 2 Jazzy Jalisco (WSL2 + WSLg)

## Paso 1: Crear el paquete del robot


```bash
mkdir folder_proyecto\src # Dentro de src donde se ubicarán los paquetes ROS2
cd folder_proyecto/src 
ros2 pkg create --build-type ament_python mi_robot #Se crea un paquete de ROS2 mi_robot con soporte para Python.
```
## Paso 2: Crear carpetas y archivos del robot

Creamos tres carpetas que organizan los archivos clave del robot:

`launch/`: contiene los archivos .launch.py que permiten iniciar nodos y configurar el sistema de forma automatizada. Es útil para lanzar múltiples nodos al mismo tiempo con sus respectivos parámetros.

`urdf/`: almacena la descripción del robot. Usualmente se utiliza el formato .urdf o .xacro, que describe los enlaces (links), articulaciones (joints) y geometría del robot.

`rviz/`: guarda configuraciones de visualización de RViz, como la posición de cámaras, marcos de referencia visibles, sensores activos, etc.

```bash
cd mi_robot
mkdir launch urdf rviz
touch launch/display.launch.py urdf/robot.urdf.xacro rviz/display.rviz
```

## Paso 3 Crear el URDF 

`URDF` (Unified Robot Description Format) es un formato basado en XML que se utiliza en ROS para describir la estructura física y cinemática de un robot. Esto incluye sus partes (enlaces o links), cómo están conectadas (articulaciones o joints), y detalles como masa, forma, color y límites de movimiento.

El URDF no define controladores ni comportamiento, sino la geometría, estructura y relaciones cinemáticas del robot. Es la base para su visualización y simulación en herramientas como RViz y Gazebo.

```xml
<!-- Estructura general del archivo URDF -->
<?xml version="1.0"?>
<robot name="mi_robot">

  <!-- Enlaces individuales del robot -->
  <link name="..."> ... </link>
  <link name="..."> ... </link>
  <link name="..."> ... </link>

  <!-- Articulaciones que unen los enlaces -->
  <joint name="..." type="..."> ... </joint>
  <joint name="..." type="..."> ... </joint>
  <joint name="..." type="..."> ... </joint>

</robot>
```
Cada `<link>` representa una parte física del robot (como una barra, un motor, una base).
Cada `<joint>` representa una articulación entre dos enlaces, con sus propiedades cinemáticas y dinámicas.

# Propiedades URDF <link>
`<inertial>`: propiedades físicas
Describe masa y momento de inercia del enlace. Es esencial para simulaciones físicas realistas.

- origin: posiciona el centro de masa.
- mass: masa del objeto en kilogramos.
- inertia: matriz de inercia (en kg·m²), describe resistencia al cambio de rotación.

```xml
  <inertial>
     <origin xyz="0 0 0.5" rpy="0 0 0"/>
     <mass value="1"/>
     <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
  </inertial>
```

`<visual>`:Se utiliza para mostrar el robot en RViz o Gazebo.(Simuladores y visualizadores)

- `<geometry>`: puede ser una forma básica (box, sphere, cylinder) o un modelo .dae/.stl.
- `<material>`: define el color y nombre del material. Se puede reutilizar entre enlaces.

```xml
  <visual>
     <origin xyz="0 0 0" rpy="0 0 0" />
     <geometry>
       <cylinder radius="1" length="0.5"/>
     </geometry>
     <material name="Cyan">
       <color rgba="0 1.0 1.0 1.0"/>
     </material>
   </visual>
```

`<collision>`:geometría para colisiones
Se utiliza para detectar colisiones en simulación. Generalmente igual al visual, pero puede simplificarse para eficiencia.

```xml
  <collision>
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <geometry>
       <cylinder radius="1" length="0.5"/>
     </geometry>
  </collision>
```
# Propiedades URDF <joint>
Define cómo se conectan dos enlaces y cómo pueden moverse entre sí.

```xml
  <joint name="my_joint" type="floating">
      <origin xyz="0 0 1" rpy="0 0 3.1416"/>
      <parent link="link1"/>
      <child link="link2"/>

      <dynamics damping="0.0" friction="0.0"/>
      <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
      <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-2.0" soft_upper_limit="0.5" />
 
  </joint>
```

- `name`: nombre único de la articulación.
- `type`: tipo de articulación (fixed, revolute, prismatic, floating, continuous, planar).
- `origin`: posición y orientación relativa entre parent y child.
- `parent / child`: nombres de los enlaces que se conectan.

Atributos adicionales:

- `limit`: restringe el movimiento angular o lineal (para revolute y prismatic).
- `dynamics`: resistencia al movimiento (amortiguamiento y fricción).
- `safety_controller`: define límites suaves y seguridad para evitar daños.

# XACRO 

Xacro (XML Macro) es una extensión del formato URDF que permite usar macros, constantes y reutilización de código. Es especialmente útil cuando un robot tiene componentes repetitivos o cuando se quieren definir parámetros reutilizables, como dimensiones o constantes físicas (por ejemplo, π).


# Ejemplo de URDF 

Plan de robot:


Crearemos un robot que tiene 3 grados de libertad (3 DoF), con tres revolutas siguiendo la descripcion a continucacion.

`base_link` → `joint1` → `link1`

`link1` → `joint2` → `link2`

`link2` → `joint3` → `link3`

```xml
<?xml version="1.0"?>
<!-- Declaración de robot y espacio de nombres de Xacro -->
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="simple_arm">

  <!-- Definición de constante: pi -->
  <xacro:property name="pi" value="3.14159"/>

  <!-- Enlace base del robot, punto fijo desde donde parte toda la estructura -->
  <link name="base_link"/>

  <!-- Articulación 1: permite rotación alrededor del eje Z -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>             <!-- Enlace padre: base -->
    <child link="link1"/>                 <!-- Enlace hijo: primer segmento -->
    <origin xyz="0 0 0.1" rpy="0 0 0"/>   <!-- Posición de la articulación respecto al padre -->
    <axis xyz="0 0 1"/>                   <!-- Eje de rotación: Z -->
    <limit lower="-${pi}" upper="${pi}" effort="1" velocity="1"/> <!-- Rango de movimiento y límites físicos -->
  </joint>

  <!-- Enlace 1: barra azul en forma de caja -->
  <link name="link1">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.3"/>               <!-- Tamaño de la caja: 10x10x30 cm -->
      </geometry>
      <origin xyz="0 0 0.15"/>                 <!-- Centrado respecto al eje Z del enlace -->
      <material name="blue">                  <!-- Color azul -->
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Articulación 2: rotación sobre eje Y (tipo codo) -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>                   <!-- Conecta el link1 -->
    <child link="link2"/>                    <!-- Con el siguiente enlace -->
    <origin xyz="0 0 0.3" rpy="0 0 0"/>       <!-- Ubicación al final del link1 -->
    <axis xyz="0 1 0"/>                       <!-- Eje de rotación: Y -->
    <limit lower="-${pi/2}" upper="${pi/2}" effort="1" velocity="1"/> <!-- Movimiento limitado a ±90° -->
  </joint>

  <!-- Enlace 2: cilindro verde que simula una parte del brazo -->
  <link name="link2">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/> <!-- Cilindro de 30 cm y radio 5 cm -->
      </geometry>
      <origin xyz="0 0 0.15"/>                <!-- Posicionado de forma centrada -->
      <material name="green">                <!-- Color verde -->
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Articulación 3: rotación sobre eje X -->
  <joint name="joint3" type="revolute">
    <parent link="link2"/>                   <!-- Conecta link2 -->
    <child link="link3"/>                    <!-- Con link3 -->
    <origin xyz="0 0 0.3" rpy="0 0 0"/>       <!-- Al final de link2 -->
    <axis xyz="1 0 0"/>                       <!-- Eje de rotación: X -->
    <limit lower="-${pi}" upper="${pi}" effort="1" velocity="1"/> <!-- Rango completo de giro -->
  </joint>

  <!-- Enlace 3: esfera roja al final del brazo -->
  <link name="link3">
    <visual>
      <geometry>
        <sphere radius="0.05"/>              <!-- Esfera con radio de 5 cm -->
      </geometry>
      <origin xyz="0 0 0.05"/>               <!-- Posicionada centrada en eje Z -->
      <material name="red">                  <!-- Color rojo -->
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

</robot>

```
## Paso 4 Crear un launch

Los archivos launch permiten iniciar múltiples nodos de ROS 2 de manera organizada y automática. En ROS2, un nodo es un proceso, una unidad de ejecución, que realiza tareas específicas dentro de un sistema robótico Son especialmente útiles cuando:

- Se requiere iniciar varios procesos (como RViz, controladores, sensores virtuales) al mismo tiempo.
- Se quiere configurar parámetros al vuelo.
- Se busca repetir una misma configuración de prueba muchas veces sin escribir comandos manuales.

Un `launch.py` típico incluye:

- Importaciones necesarias (`launch`, `launch_ros`, etc.)
- Ruta al modelo URDF/Xacro del robot.

- Nodos que deben lanzarse, como:

    * `robot_state_publisher`: publica la posición de cada articulación del robot.
    * `joint_state_publisher_gui`: interfaz para manipular las articulaciones manualmente.
    * `rviz2`: visualizador 3D.

# Ejemplo

```python
# Importa las clases necesarias para definir una descripción de lanzamiento y nodos ROS 2
from launch import LaunchDescription
from launch_ros.actions import Node

# Permite obtener la ruta al paquete desde el entorno de ROS
from ament_index_python.packages import get_package_share_directory

# Librerías estándar
import os
import xacro  # Para procesar archivos .xacro a URDF válido

def generate_launch_description():
    # Construye la ruta completa al archivo Xacro dentro del paquete 'my_robot'
    urdf_file = os.path.join(
        get_package_share_directory('my_robot'),  # Encuentra la ruta compartida del paquete
        'urdf',
        'robot.urdf.xacro'                       # Nombre del archivo Xacro
    )

    # Procesa el archivo Xacro y lo convierte a XML (formato URDF válido)
    robot_description_config = xacro.process_file(urdf_file).toxml()

    # Define y devuelve la descripción de lanzamiento con los nodos necesarios
    return LaunchDescription([

        # Nodo 1: joint_state_publisher_gui
        # Abre una interfaz gráfica con sliders para mover las articulaciones del robot
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'  # Muestra salida en la terminal
        ),

        # Nodo 2: robot_state_publisher
        # Publica la transformación entre enlaces usando la descripción URDF del robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
            # Se pasa el URDF generado desde el Xacro como parámetro
        ),

        # Nodo 3: rviz2
        # Abre RViz con una vista 3D para visualizar el modelo del robot
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])

```
## Paso 5 Configurar archivo setup

El archivo setup.py define cómo se instala y organiza un paquete de ROS 2 basado en Python. ROS 2 usa setuptools (una herramienta estándar en Python) junto con el sistema de construcción ament para empaquetar, instalar y compartir proyectos.

# Ejemplo 

```python
# Importa herramientas de instalación de paquetes en Python
from setuptools import setup

# Librerías estándar para trabajar con archivos y rutas
import os
from glob import glob  # Permite buscar múltiples archivos con comodines

# Nombre del paquete (debe coincidir con la carpeta principal)
package_name = 'my_robot'

# Configuración principal del paquete
setup(
    name=package_name,        # Nombre del paquete
    version='0.0.0',          # Versión inicial (puedes cambiarlo)
    packages=[package_name],  # Subcarpeta con archivos Python (si los hubiera)

    # Archivos que se deben instalar y compartir
    data_files=[
        ('share/' + package_name, ['package.xml']),  # Archivo de metadatos de ROS 2
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # Archivos .launch.py
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),   # Archivos URDF/Xacro
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),    # Configuraciones de RViz
    ],

    install_requires=['setuptools'],  # Dependencia para instalar
    zip_safe=True,                    # Si es seguro comprimirlo como zip (generalmente sí)

    maintainer='your_name',          # Cambiar por el nombre del autor o mantenedor
    maintainer_email='your@email.com',  # Correo de contacto
    description='Simple 3DOF articulated arm example',  # Breve descripción
    license='MIT',                   # Tipo de licencia (MIT, BSD, GPL, etc.)

    tests_require=['pytest'],       # Framework de pruebas si se usara
    entry_points={
        'console_scripts': [],      # Aquí van los scripts Python ejecutables si los hubiera
    },
)

```

## Paso 6 Editar package.xml

El archivo package.xml es un archivo de metadatos obligatorio para todo paquete ROS 2. Define la información del paquete, como:

- Nombre y versión.
- Autor y licencia.
- Dependencias (tanto de compilación como de ejecución).
- Tipo de sistema de construcción (por ejemplo, ament_python).

Es utilizado por herramientas como `colcon`, `rosdep` y `ros2 launch` para entender cómo construir, instalar y ejecutar el paquete correctamente.

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3"> <!-- Versión del formato del archivo de paquete -->

  <name>my_robot</name>        <!-- Nombre del paquete (debe coincidir con la carpeta) -->
  <version>0.0.0</version>     <!-- Versión del paquete -->
  <description>TODO: Package description</description> <!-- Descripción (debe reemplazarse) -->

  <!-- Información del mantenedor -->
  <maintainer email="oliver@todo.todo">oliver</maintainer>

  <!-- Tipo de licencia (cambia TODO por MIT, BSD, GPL, etc.) -->
  <license>TODO: License declaration</license>

  <!-- Dependencias necesarias para ejecutar pruebas (opcional en este caso) -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <!-- Exportar el tipo de sistema de construcción -->
  <export>
    <build_type>ament_python</build_type> <!-- Indica que se usa setuptools con Python -->
  </export>

  <!-- Dependencias necesarias para ejecutar el paquete -->
  <exec_depend>joint_state_publisher_gui</exec_depend> <!-- GUI para mover articulaciones -->
  <exec_depend>robot_state_publisher</exec_depend>     <!-- Publica transformaciones del robot -->
  <exec_depend>rviz2</exec_depend>                     <!-- Visualización en 3D -->

  <!-- Herramientas de construcción necesarias -->
  <buildtool_depend>ament_cmake</buildtool_depend>     <!-- Incluido por compatibilidad general -->
  <buildtool_depend>ament_python</buildtool_depend>    <!-- Necesario al usar setup.py -->

  <!-- Dependencias de código Python para localizar archivos del paquete -->
  <build_depend>ament_index_python</build_depend>
  <exec_depend>ament_index_python</exec_depend>

</package>
```

## Paso 7 Construir el paquete

Una vez configurados correctamente los archivos `setup.py` y `package.xml`, el siguiente paso es compilar el paquete usando `colcon`, la herramienta de construcción recomendada en ROS 2.

```bash
cd ~/folder_proyecto
colcon build
source install/setup.bash
```

## Paso 8 Correr el launch de nuestro robot

```bash
ros2 launch my_robot display.launch.py
```
## Paso 9 Opcional abrir la terminal para leer los datos de matriz

```bash
ros2 run tf2_ros tf2_echo base_link link3
```
