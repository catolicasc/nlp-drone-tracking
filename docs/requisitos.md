# Requisitos — Oracle Vision V2

Versão 1.0 (2026-08-15). Derivado da implementação real em `apps/isaac_app/`,
`ros2_ws/src/v2_vlm_drone/`, `scripts/` e `config/` — cada requisito cita a
evidência de rastreabilidade (arquivo/tópico/parâmetro). Status: [OK]
implementado e validado, [PARCIAL] implementado sem validação completa,
[FUTURO] não implementado.

O sistema é um pipeline **VLM-only** de busca de pessoas por drone em
simulação: o operador descreve a missão em linguagem natural; um agente com
VLM (tool-calling) planeja e executa ações determinísticas sobre MAVROS/PX4,
câmera e detector YOLO, tudo dentro do Isaac Sim/Pegasus.

---

## 1. Requisitos Funcionais (RF)

### Interface com o operador

**RF-01 — Tarefa em linguagem natural.** O operador envia a missão como texto
em linguagem natural no tópico `/v2/task`; o agente interpreta e executa.
Evidência: `v2_vlm_agent_node.py` (subscriber), `scripts/v2_send_task.sh`.
Status: [OK]

**RF-02 — Status da missão.** O agente publica o estado corrente da missão em
`/v2/status`. Evidência: `v2_vlm_agent_node.py`. Status: [OK]

**RF-03 — Diagnóstico operacional.** Comando `doctor` reporta estado dos
componentes (simulação, MAVROS, agente, câmera). Evidência:
`scripts/v2_status.sh`. Status: [OK]

### Agente VLM

**RF-04 — Planejamento via VLM com tool-calling.** Todas as decisões de missão
são tomadas por um VLM OpenAI-compatible (OpenAI, OpenRouter, Ollama) via
chamadas explícitas de tools. **Não existe fallback heurístico**: sem API
configurada, o agente falha ao iniciar com mensagem clara.
Evidência: `vlm_client.py`, `tool_defs.py`, `.env` (`LVLM_API_*`).
Status: [OK]

**RF-05 — Ferramentas determinísticas.** O VLM dispõe exclusivamente das
tools: `get_drone_status`, `inspect_camera`, `detect_person_in_frame`,
`scan_for_people`, `takeoff`, `goto_position`, `goto_relative`, `set_yaw`,
`rotate_yaw`, `hover`, `land`, `complete_task`. Cada tool é determinística e
mapeia para tópicos/serviços MAVROS ou câmera.
Evidência: `ros_tools.py`, `tool_defs.py`. Status: [OK]

**RF-06 — Memória de missão.** O agente mantém pose inicial (home), ações
recentes e achados visuais ao longo da missão.
Evidência: `memory.py` (`V2Memory`). Status: [OK]

### Segurança de voo

**RF-07 — Limites de movimento.** Toda tool de movimento valida os limites
**antes** de publicar setpoints: `min_alt_m=0.5`, `max_alt_m=10.0`,
`max_radius_m=15.0` (parâmetros ROS). Violação → recusa com mensagem
indicando o limite violado. Evidência: `ros_tools.py`, parâmetros do nó.
Status: [OK]

**RF-08 — Movimento exige MAVROS conectado.** Tools de movimento recusam
execução se o MAVROS não estiver conectado ao PX4. Exceção: `land` é sempre
permitido como comando de segurança.
Evidência: `ros_tools.py`. Status: [OK]

### Detecção de pessoas

**RF-09 — Detecção local de pessoas.** Detector YOLO (Ultralytics, classe
`person`) roda localmente sobre frames de `/drone/camera/image_raw` e publica
detecções em `/v2/yolo/detections`. Limiar de confiança configurável
(`YOLO_CONF`, default 0.35). Evidência: `yolo_detector.py`,
`yolo_person_node.py`. Status: [OK]

**RF-10 — Confirmação por VLM (opcional).** Em `scan_for_people`, detecções
YOLO podem ser confirmadas pelo VLM antes de aceitas
(`YOLO_CONFIRM_VLM=true`). Evidência: `ros_tools.py`, `.env.example`.
Status: [OK]

**RF-11 — Varredura por pessoas.** O agente executa varreduras (sequência de
`goto` + captura + detecção) para localizar pessoas e reporta ao operador.
Evidência: tool `scan_for_people`. Status: [OK]

**RF-12 — Evidências experimentais.** Frames, detecções e métricas de cada
busca são persistidos para análise posterior (diretório configurável
`YOLO_EVIDENCE_DIR`). Evidência: `evidence.py`, `experiments/`.
Status: [OK]

### Simulação

**RF-13 — Mundo simulado declarativo.** Mundo (chão, iluminação), drone,
câmera e pessoas são configurados por `config/sim.yaml`; mudanças valem no
próximo startup. Evidência: `main.py` + `config_loader.py`. Status: [OK]

**RF-14 — Drone com piloto automático real.** O drone é um quadrotor Pegasus
controlado por PX4 SITL v1.14.4 (HIL em lockstep, TCP 4560), integrado ao ROS
2 via MAVROS (UDP 18570). QGroundControl conecta sozinho via broadcast UDP
14550. Evidência: `drone_spawner.py`, `px4_launch_tool.py`,
`scripts/run_mavros_px4.sh`. Status: [OK]

**RF-15 — Câmera de bordo publicada em ROS 2.** Câmera montada no drone
publica `/drone/camera/image_raw` + `/drone/camera/camera_info` e tópicos
compatíveis RealSense (`/camera/color/*`), com resolução, frequência,
inclinação e intrínsecas configuráveis.
Evidência: `sensors.py`, Action Graph `/Graph/ROS_Camera`, `config/sim.yaml`.
Status: [OK]

**RF-16 — Pessoas no cenário.** O cenário contém `people.count` pessoas para
a missão de busca, posicionadas dentro da área configurada
(`spawn_area_half_extent_m`), com dois modos: estático (spawner próprio) ou
animado — pessoas caminhando via IRA/`isaacsim.replicator.agent` (rotina
wander 0,8–1,5 m/s com pausas; `people.animated: true`).
Evidência: `spawn/person_spawner.py`, `main.py` (`_setup_ira_people`),
`config/ira_people.yaml`, `assets/ira_ground.usda`.
Status: estático [OK]; animado [PARCIAL — integração implementada, validação
em andamento]

**RF-17 — Sinal luminoso ao encontrar pessoa.** Luz no drone pisca quando uma
pessoa é encontrada (tópico `/person_found`).
Evidência: listener em `main.py`, luz em `_ensure_drone_light`.
Status: [FUTURO — o listener existe mas não há publisher do tópico
(código morto); conectar ao fluxo de detecção]

### Infraestrutura operacional

**RF-18 — Wrappers operacionais.** Scripts `v2_start_sim.sh`,
`v2_start_mavros.sh`, `v2_agent.sh`, `v2_status.sh`, `v2_send_task.sh`
sobem e operam o sistema completo. Evidência: `scripts/`. Status: [OK]

---

## 2. Requisitos Não Funcionais (RNF)

**RNF-01 — Segurança como restrição dura.** Os limites de RF-07/RF-08 não são
negociáveis nem contornáveis pelo VLM: a validação acontece na camada
determinística (tools), fora do alcance do modelo.
Evidência: `ros_tools.py` (validação pré-publicação).

**RNF-02 — Desempenho do pipeline de visão.** A câmera publica a taxa
configurada (default 15 Hz; ~27 Hz observado em RTX 5090) suficiente para
detecção YOLO em tempo quase real durante voo.
Evidência: `config/sim.yaml` (`frequency`), medições em
`experiments/`.

**RNF-03 — Reprodutibilidade experimental.** Intrínsecas de câmera derivadas
exclusivamente do `sim.yaml` (modelo pinhole explícito, fx=fy=824.62 p/ config
atual), cenário declarativo (`.usda` versionado + config), e evidências
persistidas por missão — requisitos metodológicos para o artigo.
Evidência: `sensors.py` (`_apply_pinhole_intrinsics`), `assets/ira_ground.usda`,
`evidence.py`.

**RNF-04 — Confiabilidade sem degradação silenciosa.** Falhas de integração
(VLM offline, MAVROS desconectado, extensões ausentes) produzem mensagens
explícitas e recusa de serviço — nunca comportamento heurístico substituto.
Evidência: `vlm_client.py`, `ros_tools.py`, fallbacks com log em `main.py`.

**RNF-05 — Compatibilidade de plataforma.** Isaac Sim 6.0.0, Pegasus branch
`isaac6` (PR #144), PX4-Autopilot v1.14.4, ROS 2 Jazzy (Ubuntu 24.04),
RMW Fast DDS, GPUs NVIDIA RTX (driver ≥ 580).
Evidência: `docs/setup-maquina-nova.md`, `.env.example`.

**RNF-06 — Independência de provedor VLM.** O cliente VLM é
OpenAI-compatible: troca de provedor só exige mudar endpoint/modelo no
`.env`, sem alteração de código. Evidência: `vlm_client.py`.

**RNF-07 — Configurabilidade declarativa.** Parâmetros operacionais (mundo,
câmera, pessoas, limites de segurança, YOLO) vivem em `sim.yaml`, `.env` e
parâmetros ROS — nada de constantes mágicas no código.
Evidência: `config/`, `declare_parameter` nos nós.

**RNF-08 — Portabilidade de ambiente.** O ambiente completo é recriável em
máquina nova a partir do guia versionado (clone + submódulos + installs),
com `.env` fora do controle de versão.
Evidência: `docs/setup-maquina-nova.md`, `.gitignore`.

**RNF-09 — Segurança da informação.** Chaves de API existem apenas no `.env`
(não versionado); nenhum segredo no código ou configs do repo.
Evidência: `.env.example`, `.gitignore`.

**RNF-10 — Rastreabilidade para pesquisa.** Cada requisito funcional cita
sua evidência de implementação; evidências de experimentos são persistidas
com metadados para compor os resultados da dissertação.
Evidência: este documento, `evidence.py`, `experiments/`.

**RNF-11 — Operabilidade por não especialista.** O ciclo completo
(simulação → MAVROS → agente → tarefa) é executável por wrappers de uma
linha, com diagnóstico `doctor` e documentação de quickstart.
Evidência: `scripts/v2_*.sh`, `docs/v2_quickstart.md`.

---

## 3. Manutenção

Ao alterar comportamento coberto por um requisito aqui, atualize o requisito
e seu status na mesma mudança. Novos requisitos entram com evidência e
status.
