# PX4 SITL - Path Planner con MAVROS

Este proyecto ejecuta una simulacion PX4 SITL con Gazebo y un paquete ROS 2 (`path_planner`) para enviar una mision por waypoints al dron usando MAVROS.

La prueba se realizo usando tres terminales: una para el mundo de Gazebo/PX4, otra para MAVROS y una tercera para ejecutar la mision con los waypoints.

## Contenido principal

- `docker-compose.yml`: levanta el contenedor `px4_sitl`.
- `ros_ws/worlds/default.sdf`: mundo usado por Gazebo.
- `ros_ws/src/path_planner`: paquete ROS 2 con el planner y la mision MAVROS.
- `videos/video.mp4`: video de evidencia de la simulacion.

## Requisitos

- Docker
- Docker Compose
- Navegador web para ver la GUI por NoVNC

## Ejecucion

### 1. Construir y levantar el contenedor

```bash
./build.sh --all
docker-compose up -d
```

La interfaz grafica de Gazebo se puede abrir desde:

```text
http://localhost:6080/vnc.html
```

Password:

```text
1234
```

## Terminales necesarias

### Terminal 1 - Mundo PX4/Gazebo

En esta terminal se inicia el dron dentro del mundo de simulacion.

```bash
docker exec -it px4_sitl bash
cd /root/PX4-Autopilot
make px4_sitl gz_x500
```

Cuando Gazebo abre, debe verse el dron `x500` en el mundo cargado.

### Terminal 2 - MAVROS

En esta terminal se conecta ROS 2 con PX4 mediante MAVROS.

```bash
docker exec -it px4_sitl bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557
```

MAVROS debe quedar conectado al FCU antes de lanzar la mision.

### Terminal 3 - Mision / Waypoints

En esta terminal se compila el workspace y se ejecutan los nodos que generan y envian la ruta.

```bash
docker exec -it px4_sitl bash
cd /root/ros_ws
colcon build
source install/setup.bash
ros2 launch path_planner planner.launch.py
```

El nodo `planner_node` publica la ruta en `/planned_path` y `mavros_mission` envia los setpoints a MAVROS para que el dron siga los waypoints.

## Resultado esperado

El dron despega en modo `OFFBOARD`, sigue la ruta generada por el planner y avanza por los waypoints hasta completar la mision. La evidencia se encuentra en:

```text
videos/video.mp4
```

## Apagar la simulacion

```bash
docker-compose down
```
