# Setup de máquina nova — UAV V2 (Isaac Sim 6.0.0)

Guia completo para recriar o ambiente de desenvolvimento/simulação do projeto
em um computador novo. Foi escrito na troca de máquina de 2026-08-13; a máquina
anterior rodava Ubuntu 24.04 + RTX 5080 Laptop 16 GB.

> Fluxo de uso diário: `docs/v2_quickstart.md`.
> Erros conhecidos: `.zcode/skills/isaac-sim-projeto/references/erros-conhecidos.md`
> (no repo mestrado).

## 0. Pré-requisitos de hardware/software

- Ubuntu 24.04 x86_64
- GPU NVIDIA RTX com RT cores e **≥ 16 GB VRAM** (RTX 5080 OK; A100/H100 não suportadas)
- Driver NVIDIA **≥ 580.95.05** (mínimo oficial do Isaac Sim 6.0.0; as docs do 6.0.1
  pedem 595.58.03 — se usar 6.0.0, 580.x serve)
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
- `~/isaacsim-6.0.0` — Isaac Sim (ver seção 3)

## 2. ROS 2 Jazzy + MAVROS

```bash
sudo apt update
sudo apt install ros-jazzy-ros-base ros-dev-tools
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-msgs
sudo apt install ros-jazzy-rmw-fastrtps-cpp
# (opcional) ros-jazzy-ros2bag ros-jazzy-rqt-*
source /opt/ros/jazzy/setup.bash
```

## 3. Isaac Sim 6.0.0 (standalone)

```bash
# Download (~12.2 GB)
cd ~/Downloads
curl -L -o isaac-sim-standalone-6.0.0-linux-x86_64.zip \
  "https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-6.0.0-linux-x86_64.zip"
md5sum isaac-sim-standalone-6.0.0-linux-x86_64.zip
# esperado: 40ec5248271a0c2e7bc03f1ae725ca4c

# Instalar
mkdir -p ~/isaacsim-6.0.0
cd ~/isaacsim-6.0.0
unzip -q ~/Downloads/isaac-sim-standalone-6.0.0-linux-x86_64.zip
./post_install.sh
```

Notas:
- **Não baixar o pack de assets** (5 × ~15 GB): o 6.0 usa streaming do S3 da NVIDIA
  (`get_assets_root_path()` → `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0`).
- **Dependência extra do Pegasus** (backend MAVLink):
  ```bash
  ~/isaacsim-6.0.0/python.sh -m pip install pymavlink
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

O binário usado pelo launcher: `build/px4_sitl_default/bin/px4` (com
`build/px4_sitl_default/etc`).

## 6. QGroundControl

```bash
# AppImage do site oficial (https://qgroundcontrol.com)
wget -O ~/Apps/QGroundControl-x86_64.AppImage <URL>
chmod +x ~/Apps/QGroundControl-x86_64.AppImage
~/Apps/QGroundControl-x86_64.AppImage &
```

Com o launcher do projeto, o QGC conecta **sozinho** (broadcast UDP 14550).
Se não conectar: Comm Links → Add → UDP `127.0.0.1:14550`.

## 7. `.env` (CRÍTICO — não versionado)

O `.env` tem as chaves de API (LVLM + SPEACH_MODEL) e **não está no git**.
Copie o da máquina antiga, ou preencha do `.env.example`:

```bash
cd ~/npl-drone-mestrado/npl-drone-tracking
cp .env.example .env
# editar:
#   ISAAC_SIM_PATH=/home/<SEU_USUARIO>/isaacsim-6.0.0
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

- Câmera: `ros2 topic hz /drone/camera/image_raw` (~10 Hz; o `echo --once` do CLI
  demora no discovery — usar um subscriber rclpy com spin por 20-30 s para medir).
- Prints do app (Cena pronta, pessoas) ficam no log do Kit, não no terminal:
  `tail -f ~/isaacsim-6.0.0/kit/logs/Kit/Isaac-Sim\ Python/6.0/kit_*.log`

## Rollback (se precisar voltar ao 5.1)

- `ISAAC_SIM_PATH=/home/<SEU_USUARIO>/isaacsim` (source build 5.1) e Pegasus em
  `main`/v5.1.0 — os scripts `run_isaac.sh` e o código migrado detectam ambos
  (python.sh na raiz vs `_build/linux-x86_64/release/python.sh`; libs do bridge
  `isaacsim.ros2.core` vs `isaacsim.ros2.bridge`).
