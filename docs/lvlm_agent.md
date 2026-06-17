# Agente LVLM (ARNA-style)

Camada agentica que orquestra percepção e navegação do drone via **function calling** (OpenAI-compatible).

## Como funciona a LVLM

### Ferramentas disponíveis

| Tool | Função |
|------|--------|
| `get_drone_status` | pose, armed, mode, connected |
| `get_detections` | resumo `/person_detector/detections` |
| `inspect_camera` | análise visual (multimodal) |
| `takeoff` | OFFBOARD + arm + altitude |
| `goto_position` | setpoint x,y,z |
| `hover` | espera N segundos |
| `land` | AUTO.LAND |
| `complete_task` | encerra missão |

## Configuração

No `.env`:

```bash
LVLM_API_KEY=sk-...
LVLM_API_BASE=https://api.openai.com/v1   # ou Ollama/OpenRouter
LVLM_MODEL=gpt-4o-mini
```

Compatível com qualquer API **OpenAI-compatible** (Ollama: `http://127.0.0.1:11434/v1`).

## Uso

### Terminais

```bash
# A — simulação
./scripts/iniciar_drone.sh

# B — MAVROS
./scripts/run_mavros_px4.sh

# C — detector
./scripts/run_person_detector.sh

# D — agente
./scripts/run_lvlm_agent.sh "Decole 2m e procure pessoas"
```

### Modo heurístico (sem API)

```bash
HEURISTIC_MODE=1 ./scripts/run_lvlm_agent.sh "decolar e procurar pessoa"
```

### Tarefa via tópico ROS

```bash
ros2 run drone_agent lvlm_agent   # sem -p task
ros2 topic pub --once /drone_agent/task std_msgs/msg/String "{data: 'Pouse o drone'}"
```

### Status

```bash
ros2 topic echo /drone_agent/status
```

## Build

```bash
cd ros2_ws
colcon build --packages-select drone_agent
source install/setup.bash
```

## Referência

Inspirado em [ARNA — Agentic Robotic Navigation Architecture](https://arxiv.org/abs/2506.17462).
