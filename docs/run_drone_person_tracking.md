# Executar a aplicação (Isaac Sim + PX4 + MAVROS + Detector + Controle)

Este documento descreve o passo a passo para executar a aplicação de **drone + detecção de pessoa + tracking/busca**.

## Visão geral (o que roda)

- **Isaac Sim** (com ROS2 Bridge) roda o cenário, o drone e o PX4 SITL.
- **MAVROS** faz a ponte **MAVLink <-> ROS2** e publica tópicos como `/mavros/state`.
- **person_detector** recebe a imagem da câmera e publica `/person_detector/detections`.
- **drone_search_track** publica setpoints para o PX4 via `/mavros/setpoint_position/local`.
  - `search_and_track`: busca + yaw + hover no detect.
  - `cfc_controller`: policy (heurística ou LNN) + safety layer.

## Pré-requisitos

- `.env` configurado (principalmente `ISAAC_SIM_PATH`).
- ROS 2 instalado (no projeto existe script para **jazzy** no MAVROS e `run_ros.sh` usa `ROS_DISTRO` padrão `humble`).
- Workspace ROS2 já buildado ao menos 1 vez.

## 0) Build do workspace

Em um terminal:

```bash
cd ~/Apps/meu-projeto/ros2_ws
colcon build --symlink-install
```

Em qualquer terminal que for rodar nós ROS2 deste workspace:

```bash
source ~/Apps/meu-projeto/ros2_ws/install/setup.bash
```

## 1) Terminal A — Subir Isaac Sim (com ROS2 bridge)

```bash
cd ~/Apps/meu-projeto
./scripts/run_isaac.sh
```

Deixe esse terminal rodando.

## 2) Terminal B — Subir MAVROS apontando para o PX4

```bash
cd ~/Apps/meu-projeto
./scripts/run_mavros_px4.sh
```

Verifique se o MAVROS está publicando:

```bash
ros2 topic list | grep mavros | head
ros2 topic echo --once /mavros/state
```

Você deve ver `/mavros/state` e `connected: true`.

## 3) Terminal C — Rodar o detector de pessoa

### Rodar

O detector pode precisar do workaround de NumPy/cv_bridge. Use:

```bash
source ~/Apps/meu-projeto/ros2_ws/install/setup.bash
PYTHONNOUSERSITE=1 ros2 run person_detector hog_person_detector
```

### Checar se está publicando

Em outro terminal (ou neste, se quiser interromper com Ctrl+C depois):

```bash
PYTHONNOUSERSITE=1 ros2 topic echo --once /person_detector/detections
```

- Se aparecer `detections: []`, o nó está rodando, mas **não detectou** naquele frame.
- Se aparecer itens dentro de `detections`, detectou.

## 4) Terminal D — Rodar o controlador do drone

### Opção 1: `search_and_track` (baseline)

```bash
source ~/Apps/meu-projeto/ros2_ws/install/setup.bash
ros2 run drone_search_track search_and_track --ros-args \
  -p search_radius_max:=5.0
```

### Opção 2: `cfc_controller` (policy + safety)

#### 2.1 Rodar em modo heurístico (sem LNN)

```bash
source ~/Apps/meu-projeto/ros2_ws/install/setup.bash
ros2 run drone_search_track cfc_controller --ros-args \
  -p use_lnn:=false \
  -p policy_mode:=tracking_only \
  -p search_radius_max:=5.0 \
  -p takeoff_z:=2.0
```

#### 2.2 Rodar em modo LNN (precisa torch + modelo TorchScript)

1) Garanta que o `torch` está instalado no **mesmo Python** que executa o ROS2:

```bash
python3 -c "import torch; print(torch.__version__)"
```

2) Rode com `use_lnn:=true` e `model_path` apontando para um arquivo real:

```bash
source ~/Apps/meu-projeto/ros2_ws/install/setup.bash
ros2 run drone_search_track cfc_controller --ros-args \
  -p use_lnn:=true \
  -p model_path:="/caminho/real/para/modelo_cfc.ts.pt" \
  -p policy_mode:=tracking_only \
  -p search_radius_max:=5.0 \
  -p takeoff_z:=2.0
```

Observações:

- Se `model_path` estiver vazio/inválido, o nó **cai no heurístico**.
- Não use `PYTHONNOUSERSITE=1` no `cfc_controller` se você instalou `torch` via `pip --user`.

## 5) Sinais para confirmar que está funcionando

### MAVROS/PX4

- Estado do PX4:

```bash
ros2 topic echo --once /mavros/state
```

Você quer ver:

- `connected: true`
- `mode: OFFBOARD` (depois que o controlador estiver rodando)
- `armed: true`

### Setpoints

```bash
ros2 topic echo --once /mavros/setpoint_position/local
```

### Detecção e “person_found”

- Detecções:

```bash
PYTHONNOUSERSITE=1 ros2 topic echo --once /person_detector/detections
```

- Flag booleana de detecção fresca:

```bash
ros2 topic echo /person_found
```

### Indicador visual no Isaac

- Quando `/person_found` fica `true`, a luz no drone deve piscar.

## Troubleshooting

### A) `/mavros/state` não aparece

- Confirme se MAVROS está rodando:

```bash
ros2 node list | grep mavros
ros2 topic list | grep mavros | head
```

- Confirme que Isaac/PX4 está rodando e que `run_mavros_px4.sh` está apontando para a porta correta (default do projeto: `udp://@127.0.0.1:18570`).

### B) `hog_person_detector` quebra com erro de NumPy/cv_bridge

- Rode com:

```bash
PYTHONNOUSERSITE=1 ros2 run person_detector hog_person_detector
```

Isso evita pegar pacotes do `~/.local`.

### C) `cfc_controller`: `No module named 'torch'`

Isso significa que o `torch` não está instalado no Python que o ROS2 está usando (`/usr/bin/python3`).

- Verifique:

```bash
python3 -c "import sys; print(sys.executable)"
python3 -c "import torch; print(torch.__version__)"
```

- Instale (CPU):

```bash
python3 -m pip install --user torch --index-url https://download.pytorch.org/whl/cpu
```

Depois reabra o terminal (ou garanta que o `~/.local` está no path do Python).

### D) `cfc_controller`: `LNN enabled but model_path is empty`

- Você ligou `use_lnn:=true`, mas não passou `model_path`.
- Passe um caminho real para um modelo TorchScript.

### E) `/person_detector/detections` “não está sendo publicado”

- Significa que o nó do detector **não está rodando** (ou caiu).
- Reexecute o detector e confira `ros2 node list | grep person`.
