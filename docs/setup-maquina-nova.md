# Setup de máquina nova — UAV V2 (Isaac Sim 6.0)

Guia completo para recriar o ambiente de desenvolvimento/simulação do projeto
em um computador novo. Foi escrito na troca de máquina de 2026-08-13; a máquina
anterior rodava Ubuntu 24.04 + RTX 5080 Laptop 16 GB. Validado de novo em
2026-08-15 no desktop com Isaac Sim **6.0.1** (`~/isaac-sim`, RTX 5090, driver
610.43.02): PX4 SITL + lockstep, câmera a ~16 Hz e broadcast QGC na 14550 OK.

> Fluxo de uso diário: `docs/v2_quickstart.md`.
> Erros conhecidos: `.zcode/skills/isaac-sim-projeto/references/erros-conhecidos.md`
> (no repo mestrado).

## 0. Pré-requisitos de hardware/software

- Ubuntu 24.04 x86_64
- GPU NVIDIA RTX com RT cores e **≥ 16 GB VRAM** (RTX 5080 OK; A100/H100 não suportadas)
- Driver NVIDIA **≥ 595.58.03** (mínimo oficial do Isaac Sim 6.0.1; com 6.0.0,
  580.x serve)
- RAM ≥ 32 GB (recomendado), disco ≥ 100 GB livres
- Chave SSH configurada no GitHub (repos privados `catolicasc/*`)

## 1. Clonar os repositórios (com submodules)

```bash
git clone --recursive git@github.com:catolicasc/npl-drone-mestrado.git
cd npl-drone-mestrado
git submodule update --init --recursive
cd npl-drone-tracking
git checkout v2          # branch de trabalho (a main do submodule é antiga)
```

Estrutura resultante:
- `npl-drone-mestrado/npl-drone-tracking` — código do sistema
- `npl-drone-mestrado/npl-drone-artigo` — artigo LaTeX
- `~/PegasusSimulator` — Pegasus (ver seção 4)
- `~/PX4-Autopilot` — PX4 (ver seção 5)
- `~/isaac-sim` — Isaac Sim 6.0.1 (ver seção 3)

## 2. ROS 2 Jazzy + MAVROS

```bash
sudo apt update
sudo apt install ros-jazzy-ros-base ros-dev-tools
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-msgs
sudo apt install ros-jazzy-rmw-fastrtps-cpp
# (opcional) ros-jazzy-ros2bag ros-jazzy-rqt-*
source /opt/ros/jazzy/setup.bash
```

> **Pop!_OS (testado em 24.04 / noble, 2026-08-14):** o repo do ROS 2 **não vem
> configurado** (`Impossível encontrar o pacote ros-jazzy-*`) e o pacote pyserial
> chama-se **`python3-serial`** (não `python3-pyserial`). Configurar antes:
>
> ```bash
> sudo apt-get update && sudo apt-get install -y curl gnupg
> sudo install -d -m 0755 /etc/apt/keyrings
> sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
>   -o /etc/apt/keyrings/ros-archive-keyring.gpg
> echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
>   | sudo tee /etc/apt/sources.list.d/ros2.list
> sudo apt-get update
> ```

## 3. Isaac Sim 6.0 (standalone; testado no 6.0.1)

```bash
# Download (~12.2 GB) — 6.0.1 (ou 6.0.0, que também funciona)
cd ~/Downloads
curl -L -o isaac-sim-standalone-6.0.1-linux-x86_64.zip \
  "https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-6.0.1-linux-x86_64.zip"

# Instalar — o diretório pode ter qualquer nome; ex. da máquina atual: ~/isaac-sim
mkdir -p ~/isaac-sim
cd ~/isaac-sim
unzip -q ~/Downloads/isaac-sim-standalone-6.0.1-linux-x86_64.zip
./post_install.sh
```

Notas:
- **Não baixar o pack de assets** (5 × ~15 GB): o 6.0 usa streaming do S3 da NVIDIA
  (`get_assets_root_path()` → `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0`).
- **Dependência extra do Pegasus** (backend MAVLink):
  ```bash
  ~/isaac-sim/python.sh -m pip install pymavlink
  ```
- **1ª execução demora 10-30 min** compilando shaders RTX (RTX 50-series); não é hang —
  deixe completar. Execuções seguintes sobem em ~10-30 s.
- `python.sh` fica na **raiz** da instalação (no 5.1 source build era
  `_build/linux-x86_64/release/python.sh`).

## 4. Pegasus Simulator — branch `isaac6` (Isaac Sim 6.0)

O suporte a Isaac Sim 6.0 vive no **PR #144** (aberto, não mergeado; base `dev_6.0.1`).
O projeto pinou o head do PR. Recriar a branch:

```bash
git clone https://github.com/PegasusSimulator/PegasusSimulator.git
cd PegasusSimulator
git fetch origin refs/pull/144/head:refs/remotes/origin/pr-144
git checkout -b isaac6 fcb99c061460
```

Ajuste local do `configs.yaml` (extensions/pegasus.simulator/config/configs.yaml):

```yaml
px4_default_airframe: gazebo-classic_iris   # 'iris' se PX4 < v1.14
px4_dir: /home/<SEU_USUARIO>/PX4-Autopilot
```

> ⚠️ O launcher PX4 do Pegasus (5.1 e PR) usa `ROMFS/px4fmu_common` e **falha com
> PX4 1.14.4** (HIL não flui: `poll timeout` contínuo, EKF nunca converge). O projeto
> contorna com o monkey-patch em `apps/isaac_app/standalone/spawn/px4_launch_tool.py`
> (usa `build/px4_sitl_default/etc` + broadcast QGC na 14550) — já é o padrão do app.

## 5. PX4 Autopilot v1.14.4 + build SITL

```bash
git clone --recursive --branch v1.14.4 https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
make px4_sitl_default        # build demorado (minutos)
```

> **Python do build (testado em Pop!_OS 24.04, 2026-08-14):** o cmake do PX4
> falha com `kconfiglib is not installed` se os requisitos Python não estiverem
> presentes. O arquivo é `Tools/setup/requirements.txt` (não existe
> `requirements.txt` na raiz). No Ubuntu 24.04 o pip do sistema é
> externally-managed (PEP 668) → usar `--break-system-packages` (instala em
> `~/.local`, sem sudo) e exportar `~/.local/bin` no PATH:
>
> ```bash
> pip3 install --user --break-system-packages -r Tools/setup/requirements.txt
> export PATH="$HOME/.local/bin:$PATH"   # kconfiglib & cia. instalam scripts aqui
> ```
>
> **GCC 13 não compila o v1.14.4** (Ubuntu 24.04 vem com gcc 13 por padrão):
> falso positivo `-Werror=array-bounds` em `src/lib/matrix/matrix/Matrix.hpp`
> (e mais erros em cadeia). Instalar e usar o GCC 12:
>
> ```bash
> sudo apt install -y gcc-12 g++-12
> make distclean
> CC=gcc-12 CXX=g++-12 make px4_sitl_default
> ```

O binário usado pelo launcher: `build/px4_sitl_default/bin/px4` (com
`build/px4_sitl_default/etc`).

## 6. QGroundControl

Ubuntu 24.04 (testado em Pop!_OS 24.04, 2026-08-15):

```bash
# Dependências do AppImage (libfuse2 e libxcb-cursor0 costumam faltar no 24.04)
sudo apt install -y libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0

mkdir -p ~/Apps
curl -sL -o ~/Apps/QGroundControl-x86_64.AppImage \
  "https://d176tv9ibo4jno.cloudfront.net/builds/master/QGroundControl-x86_64.AppImage"
chmod +x ~/Apps/QGroundControl-x86_64.AppImage
~/Apps/QGroundControl-x86_64.AppImage &
```

(Opcional, só relevante para hardware serial real: `sudo usermod -aG dialout
"$(id -un)"` e desativar o ModemManager. Para o SITL via UDP não é preciso.)

Com o launcher do projeto, o QGC conecta **sozinho** (broadcast UDP 14550).
Se não conectar: Comm Links → Add → UDP `127.0.0.1:14550`.

> **Pessoas animadas (IRA) em headless**: no 6.0.1 o bake do NavMesh retorna
> mesh vazia em headless (CPU e GPU) e os personagens nascem na origem sem
> wander. Para headless use `people.animated: false` (estáticas — validado);
> o modo animado requer GUI (`HEADLESS` indefinido). Detalhes no
> `erros-conhecidos.md` da skill isaac-sim-projeto (Achado 7).

> **Entre dois runs**, derrube sobras (o python do Isaac atravessa o TERM do
> `timeout` e segura a porta TCP 4560, causando `Address already in use` no run
> seguinte):
> ```bash
> pkill -9 -f "isaac_app/standalone/main.py"; pkill -9 px4
> ```

## 7. `.env` (CRÍTICO — não versionado)

O `.env` tem as chaves de API (LVLM + SPEACH_MODEL) e **não está no git**.
Copie o da máquina antiga, ou preencha do `.env.example`:

```bash
cd ~/npl-drone-mestrado/npl-drone-tracking
cp .env.example .env
# editar:
#   ISAAC_SIM_PATH=/home/<SEU_USUARIO>/isaac-sim
#   PEGASUS_PATH=/home/<SEU_USUARIO>/PegasusSimulator
#   PX4_PATH=/home/<SEU_USUARIO>/PX4-Autopilot
#   LVLM_API_KEY=...  SPEACH_MODEL=...   (copiar da máquina antiga)
```

## 8. Venv YOLO (recriado na 1ª execução)

`scripts/lib/yolo_env.sh` usa `PROJECT_ROOT/.python_deps/yolo-venv` (ou legado
`.python_deps/yolo`); `yolov8n.pt` já está versionado no repo.

## 9. Verificação rápida

```bash
cd ~/npl-drone-mestrado/npl-drone-tracking
./scripts/v2_start_sim.sh          # aguardar "Ready for takeoff!" do PX4
./scripts/v2_start_mavros.sh       # ros2 topic echo --once /mavros/state → connected: true
```

- Câmera: `ros2 topic hz /drone/camera/image_raw` (medido ~16-18 Hz no 6.0.1 +
  RTX 5090; o `echo --once` do CLI demora no discovery — usar um subscriber rclpy
  com spin por 20-30 s para medir).
- Prints do app (Cena pronta, pessoas) só aparecem no terminal com
  `PYTHONUNBUFFERED=1`; sem ele, ficam no log do Kit:
  `tail -f ~/isaac-sim/kit/logs/Kit/Isaac-Sim\ Python/6.0/kit_*.log`

## Rollback (se precisar voltar ao 5.1)

- `ISAAC_SIM_PATH=/home/<SEU_USUARIO>/isaacsim` (source build 5.1) e Pegasus em
  `main`/v5.1.0 — os scripts `run_isaac.sh` e o código migrado detectam ambos
  (python.sh na raiz vs `_build/linux-x86_64/release/python.sh`; libs do bridge
  `isaacsim.ros2.core` vs `isaacsim.ros2.bridge`).
