# ROS2 + MAVROS (PX4) — Armar e Voar (OFFBOARD)

## Pré-requisitos

- Isaac Sim + Pegasus + PX4 rodando (ex.: `./scripts/run_isaac.sh`).
- ROS 2 Jazzy instalado (ou o distro que você estiver usando).
- MAVROS instalado.

## Visão geral

O fluxo para voar sem QGroundControl é:

1. Subir a simulação (PX4 precisa estar rodando e emitindo MAVLink).
2. Subir o MAVROS apontando para o PX4 (porta UDP 18570 neste projeto).
3. Publicar setpoints (20 Hz) por alguns segundos.
4. Trocar modo para `OFFBOARD`.
5. Armar.
6. Continuar publicando setpoints (senão o PX4 sai do OFFBOARD).

## Passo-a-passo (manual)

### 1) Subir MAVROS

Em um terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch mavros px4.launch fcu_url:=udp://@127.0.0.1:18570
```

### 2) Verificar conexão

Em outro terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /mavros/state
```

Confirme `connected: true`.

### 3) Publicar setpoint de posição (takeoff)

Em outro terminal, publique a posição desejada (ex.: z=1.5m) a 20 Hz:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub -r 20 /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped "{\
  header: {frame_id: 'map'},\
  pose: {position: {x: 0.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}\
}"
```

Deixe rodando.

### 4) Entrar em OFFBOARD

Em outro terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'OFFBOARD'}"
```

### 5) Armar

```bash
source /opt/ros/jazzy/setup.bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

### 6) Mover um pouco

Opção A: mude o setpoint de posição (ex.: x=1.0)

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub -r 20 /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped "{\
  header: {frame_id: 'map'},\
  pose: {position: {x: 1.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}\
}"
```

Opção B: velocidade (se preferir)

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub -r 20 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

### 7) Pousar

```bash
source /opt/ros/jazzy/setup.bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'AUTO.LAND'}"
```

Opcional: desarmar

```bash
source /opt/ros/jazzy/setup.bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

## Scripts

Veja os scripts em `scripts/`:

- `run_mavros_px4.sh`
- `arm_drone.sh`
- `offboard_takeoff.sh`
- `land_drone.sh`
