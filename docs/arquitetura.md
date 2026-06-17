# Arquitetura Oracle Vision

```mermaid
---
config:
  layout: elk
---
flowchart LR
    %% Paleta de cores
    classDef indigo stroke:#818cf8,fill:#eef2ff
    classDef teal stroke:#2dd4bf,fill:#f0fdfa
    classDef orange stroke:#fb923c,fill:#fff7ed
    classDef amber stroke:#fbbf24,fill:#fffbeb
    classDef violet stroke:#a78bfa,fill:#f5f3ff
    classDef green stroke:#4ade80,fill:#f0fdf4

    %% Entrada principal
    U(["Tarefa em linguagem natural<br/>ex.: Decole 2m e procure pessoas"]):::indigo

    %% Agente LVLM
    subgraph AGENT["🧠 LVLM Agent Node"]
        direction TB
        MEM[("📘 Memória<br/>ações · achados · pose home")]
        LLM["🤖 LVLM<br/>planeja e escolhe ferramentas"]
        LOOP{"🔁 Missão concluída?"}
    end
    class AGENT indigo

    %% Ferramentas
    subgraph TOOLS["🧩 Ferramentas (function calling)"]
        direction TB
        P1["get_drone_status · get_detections<br/>inspect_camera"]
        P2["takeoff · goto_position · hover · land"]
        P3["complete_task"]
    end
    class TOOLS teal

    %% Sistema ROS
    subgraph ROS["🔧 ROS 2 / Simulação"]
        direction TB
        CAM["📷 /drone/camera/image_raw"]
        PD["🕵️ person_detector<br/>hog_person_detector"]
        DET["🎯 /person_detector/detections"]
        MV["🛠 MAVROS → PX4"]
        SP["➡️ /mavros/setpoint_position/local<br/>20 Hz"]
    end
    class ROS orange

    %% API e saída
    API["🌐 API LVLM<br/>OpenAI · Ollama · OpenRouter"]:::violet
    OUT(["📡 /drone_agent/status"]):::green

    %% Fluxos
    U --> LLM
    LLM <-->|"chat + tools"| API
    LLM <-->|"leitura / escrita"| MEM

    %% Loop do agente
    LLM -->|"invoca tool"| TOOLS
    P3 --> LOOP
    LOOP -->|"não"| LLM
    LOOP -->|"sim"| OUT

    %% Percepção
    CAM --> PD --> DET
    P1 --> CAM & DET & MV

    %% Visão multimodal
    P1 -.->|"frame JPEG"| API

    %% Ação e controle
    P2 --> MV
    P2 --> SP
    SP --> MV

    %% Retorno à memória
    TOOLS -.->|"resultado JSON"| MEM
```

## Scripts de entrada

| Script | Função |
|--------|--------|
| `iniciar_drone.sh` | Isaac Sim + Pegasus + PX4 |
| `run_mavros_px4.sh` | Ponte ROS ↔ PX4 |
| `run_person_detector.sh` | Detecção de pessoas (HOG) |
| `run_lvlm_agent.sh` | Agente LVLM + navegação |
| `voar_drone.sh` | Decolagem OFFBOARD simples |
| `land_drone.sh` | AUTO.LAND |
| `check_mavlink_ports.sh` | Diagnóstico de portas |

> **Nota:** use apenas **um** controlador por vez (`voar_drone`, `lvlm_agent`, `search_and_track` ou `cfc_controller`) — todos publicam setpoints no MAVROS.
