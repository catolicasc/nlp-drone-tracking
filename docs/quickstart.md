# Quickstart

Guia rapido para configurar o Oracle Vision depois de clonar o projeto em uma maquina nova.

Este projeto assume que Isaac Sim, Pegasus Simulator e PX4 Autopilot ficam instalados fora do repositorio. O repositorio contem a aplicacao Isaac/Pegasus, o workspace ROS 2 e os scripts de orquestracao.

## Ambiente Testado

- Pop!_OS 24.04 / Ubuntu Noble
- ROS 2 Jazzy
- Isaac Sim em `/home/luiz/isaacsim`
- Pegasus Simulator em `/home/luiz/PegasusSimulator`
- PX4 Autopilot em `/home/luiz/PX4-Autopilot`

## 1. Instalar Dependencias Base

```bash
sudo apt update
sudo apt install curl gnupg lsb-release software-properties-common
sudo add-apt-repository universe
```

No Pop!_OS 24.04, o pacote `python3-colcon-common-extensions` pode nao existir. Instale o `colcon` e as extensoes necessarias assim:

```bash
sudo apt install colcon python3-colcon-ros python3-colcon-cmake python3-colcon-python-setup-py
```

## 2. Adicionar o Repositorio ROS 2

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

```bash
sudo apt update
```

Confirme que o ROS Jazzy aparece no `apt`:

```bash
apt-cache search '^ros-jazzy-desktop$'
```

## 3. Instalar ROS 2, MAVROS e Mensagens

```bash
sudo apt install ros-jazzy-desktop ros-dev-tools \
  ros-jazzy-mavros ros-jazzy-mavros-extras \
  ros-jazzy-cv-bridge ros-jazzy-vision-msgs
```

Se aparecer erro envolvendo `python3-catkin-pkg-modules`, repare o conflito entre o pacote do Ubuntu/Pop e o pacote do ROS:

```bash
sudo apt install -o Dpkg::Options::="--force-overwrite" \
  python3-catkin-pkg=1.1.0-101 python3-catkin-pkg-modules
```

Depois finalize os pacotes pendentes:

```bash
sudo apt --fix-broken install
sudo dpkg --configure -a
```

Instale os datasets do MAVROS:

```bash
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

Valide a instalacao:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
ros2 pkg list | grep mavros
ros2 pkg list | grep cv_bridge
ros2 pkg list | grep vision_msgs
```

## 4. Configurar o `.env`

Crie o `.env` a partir do exemplo:

```bash
cp .env.example .env
```

Configure os caminhos conforme sua maquina:

```bash
ISAAC_SIM_PATH=/home/luiz/isaacsim
PEGASUS_PATH=/home/luiz/PegasusSimulator
PX4_PATH=/home/luiz/PX4-Autopilot
ROS_WS_PATH=./ros2_ws

ROS_DISTRO=jazzy
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Confirme os caminhos principais:

```bash
test -x /home/luiz/isaacsim/_build/linux-x86_64/release/python.sh && echo "Isaac OK"
test -d /home/luiz/PegasusSimulator && echo "Pegasus OK"
test -d /home/luiz/PX4-Autopilot && echo "PX4 OK"
```

## 5. Instalar Dependencias do Pegasus no Python do Isaac

O Pegasus roda dentro do Python do Isaac Sim. Por isso, dependencias como `pymavlink` precisam estar instaladas nesse Python, nao apenas no Python do sistema.

```bash
cd /home/luiz/Apps/nlp-drone-tracking
set -a && source .env && set +a
"$ISAAC_SIM_PATH/_build/linux-x86_64/release/python.sh" -m pip install pymavlink
```

## 6. Buildar o Workspace ROS 2

Na raiz do projeto:

```bash
cd /home/luiz/Apps/nlp-drone-tracking
./scripts/build_ros.sh
```

Se preferir executar manualmente:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/luiz/Apps/nlp-drone-tracking/ros2_ws
colcon build --symlink-install
```

## 7. Subir a Simulacao

Use terminais separados.

Terminal A: Isaac Sim + Pegasus + PX4:

```bash
cd /home/luiz/Apps/nlp-drone-tracking
./scripts/iniciar_drone.sh
```

Espere a cena carregar no Isaac Sim e deixe a timeline em `PLAY`.

Terminal B: MAVROS:

```bash
cd /home/luiz/Apps/nlp-drone-tracking
./scripts/run_mavros_px4.sh
```

Terminal C: verificar conexao:

```bash
source /opt/ros/jazzy/setup.bash
source /home/luiz/Apps/nlp-drone-tracking/ros2_ws/install/setup.bash
ros2 topic echo --once /mavros/state
```

O campo esperado e:

```text
connected: true
```

## 8. Primeiro Voo Simples

Com Isaac/PX4 e MAVROS rodando:

```bash
cd /home/luiz/Apps/nlp-drone-tracking
./scripts/voar_drone.sh
```

Para pousar:

```bash
./scripts/land_drone.sh
```

## 9. Detector e Agente LVLM

Terminal C: detector de pessoas:

```bash
cd /home/luiz/Apps/nlp-drone-tracking
./scripts/run_person_detector.sh
```

Terminal D: agente em modo heuristico, sem API:

```bash
cd /home/luiz/Apps/nlp-drone-tracking
HEURISTIC_MODE=1 ./scripts/run_lvlm_agent.sh "Decole 2m e procure pessoas"
```

Para usar uma API OpenAI-compatible, configure no `.env`:

```bash
LVLM_API_KEY=sk-...
LVLM_API_BASE=https://api.openai.com/v1
LVLM_MODEL=gpt-4o-mini
```

Depois rode:

```bash
./scripts/run_lvlm_agent.sh "Decole 2m e procure pessoas"
```

## Troubleshooting

### `Unable to locate package python3-colcon-common-extensions`

No Pop!_OS 24.04, instale:

```bash
sudo apt install colcon python3-colcon-ros python3-colcon-cmake python3-colcon-python-setup-py
```

### `apt-cache search '^ros-jazzy-desktop$'` nao mostra nada

O repositorio ROS 2 ainda nao foi adicionado ou o `apt update` falhou. Confira:

```bash
test -f /etc/apt/sources.list.d/ros2.list && echo "repo ROS OK"
test -f /usr/share/keyrings/ros-archive-keyring.gpg && echo "keyring ROS OK"
sudo apt update
```

### `python3-catkin-pkg-modules` falha no `dpkg`

Repare o conflito de pacote:

```bash
sudo apt install -o Dpkg::Options::="--force-overwrite" \
  python3-catkin-pkg=1.1.0-101 python3-catkin-pkg-modules
sudo apt --fix-broken install
sudo dpkg --configure -a
```

### `ros2 --version` falha

Isso pode ser normal. Use:

```bash
ros2 --help
ros2 pkg list | grep mavros
```

### `ModuleNotFoundError: No module named 'pymavlink'`

Instale o pacote no Python do Isaac:

```bash
set -a && source .env && set +a
"$ISAAC_SIM_PATH/_build/linux-x86_64/release/python.sh" -m pip install pymavlink
```

### `/mavros/state` nao mostra `connected: true`

Verifique:

```bash
./scripts/check_mavlink_ports.sh
ros2 topic list | grep mavros
ros2 topic echo --once /mavros/state
```

Confirme que a simulacao esta rodando, a timeline do Isaac esta em `PLAY` e o MAVROS esta usando a porta padrao do projeto: `udp://@127.0.0.1:18570`.
