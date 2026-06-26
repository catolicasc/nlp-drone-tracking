# V2 Quickstart

## 1. Configure o `.env`

Copie `.env.example` para `.env` se ainda não existir e preencha pelo menos:

```bash
ISAAC_SIM_PATH=/caminho/para/isaacsim
PX4_PATH=/caminho/para/PX4-Autopilot
LVLM_API_BASE=https://api.openai.com/v1
LVLM_API_KEY=sk-...
LVLM_MODEL=gpt-4o-mini
```

Para Ollama/local compatível com `/v1/chat/completions`:

```bash
LVLM_API_BASE=http://127.0.0.1:11434/v1
LVLM_API_KEY=ollama
LVLM_MODEL=qwen2.5vl:latest
```

## 2. Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Apps/meu-projeto/ros2_ws
colcon build --packages-select v2_vlm_drone
source install/setup.bash
```

## 3. Terminais

Terminal 1, Isaac/PX4:

```bash
cd ~/Apps/meu-projeto
./scripts/v2_start_sim.sh
```

Espere o Isaac abrir, pressione Play e aguarde PX4 estabilizar.

Terminal 2, MAVROS:

```bash
cd ~/Apps/meu-projeto
./scripts/v2_start_mavros.sh
```

Verifique conexão:

```bash
ros2 topic echo --once /mavros/state
```

Terminal 3, agente V2:

```bash
cd ~/Apps/meu-projeto
./scripts/v2_agent.sh
```

Terminal 4, status:

```bash
cd ~/Apps/meu-projeto
./scripts/v2_status.sh doctor
```

O `doctor` precisa mostrar pelo menos:

- um nó `v2_vlm_agent`;
- `/v2/task` com 1 assinante;
- `/mavros/state` com `connected: true`;
- `/drone/camera/image_raw` publicado se você for usar visão.

Terminal 5, comandos:

```bash
cd ~/Apps/meu-projeto
./scripts/v2_send_task.sh "decole 1 metro, avance 1 metro, fique parado 3 segundos e pouse"
```

## Diagnóstico rápido

```bash
./scripts/v2_status.sh doctor
./scripts/v2_status.sh state
./scripts/v2_status.sh pose
./scripts/v2_status.sh statustext
```

Se `./scripts/v2_send_task.sh` disser que não existe assinante em `/v2/task`, o agente V2 não está rodando ou está em outro `ROS_DOMAIN_ID`. Todos os scripts V2 carregam `.env`; mantenha o mesmo `ROS_DOMAIN_ID` para Isaac, MAVROS, agente e envio de tarefa.

Se o agente não iniciar, normalmente falta `LVLM_API_KEY` no `.env`. A V2 não ativa heurística automaticamente.
