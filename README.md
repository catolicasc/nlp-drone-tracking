# Oracle Vision V2

Pipeline VLM-only para controlar um drone no Isaac Sim/Pegasus/PX4 via ROS 2 e MAVROS.

## Estrutura

- `apps/isaac_app/standalone`: simulação Isaac/Pegasus e publicação da câmera.
- `config/sim.yaml`: mundo, drone, câmera e pessoas da simulação.
- `ros2_ws/src/v2_vlm_drone`: agente V2 VLM-only.
- `scripts/v2_*.sh`: wrappers operacionais da V2.
- `docs/v2_*.md`: documentação da V2.

## Quickstart

Configure `.env` a partir de `.env.example`, depois siga:

```bash
docs/v2_quickstart.md
```

Fluxo principal:

```bash
./scripts/v2_start_sim.sh
./scripts/v2_start_mavros.sh
./scripts/v2_agent.sh
./scripts/v2_status.sh doctor
./scripts/v2_send_task.sh "decole 1 metro, procure pessoas e me avise quando encontrar"
```

## Documentação

- `docs/v2_architecture.md`
- `docs/v2_camera.md`
- `docs/v2_commands.md`
- `docs/v2_quickstart.md`
