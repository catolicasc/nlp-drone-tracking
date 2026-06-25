"""Definições JSON Schema das ferramentas do agente."""

from __future__ import annotations

from typing import Any


def tool_schemas() -> list[dict[str, Any]]:
    return [
        {
            "type": "function",
            "function": {
                "name": "get_drone_status",
                "description": (
                    "Retorna pose local, modo de voo, armed e se MAVROS está conectado ao PX4."
                ),
                "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
            },
        },
        {
            "type": "function",
            "function": {
                "name": "get_detections",
                "description": (
                    "Retorna resumo das detecções de pessoas na câmera "
                    "(contagem, melhor detecção, posição normalizada na imagem)."
                ),
                "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
            },
        },
        {
            "type": "function",
            "function": {
                "name": "inspect_camera",
                "description": (
                    "Analisa o último frame da câmera (visão). Use para confirmar cena visualmente."
                ),
                "parameters": {
                    "type": "object",
                    "properties": {
                        "question": {
                            "type": "string",
                            "description": "Pergunta sobre a imagem (ex.: há pessoas visíveis?)",
                        }
                    },
                    "required": ["question"],
                    "additionalProperties": False,
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "takeoff",
                "description": (
                    "Publica setpoints, entra em OFFBOARD e arma o drone na altitude z (metros)."
                ),
                "parameters": {
                    "type": "object",
                    "properties": {
                        "z": {"type": "number", "description": "Altitude alvo em metros (ex.: 2.0)"},
                    },
                    "required": ["z"],
                    "additionalProperties": False,
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "goto_position",
                "description": "Move o drone para x,y,z no frame local map e mantém por alguns segundos.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "z": {"type": "number"},
                        "hold_sec": {
                            "type": "number",
                            "description": "Segundos para manter o setpoint (default 4)",
                        },
                    },
                    "required": ["x", "y", "z"],
                    "additionalProperties": False,
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "hover",
                "description": "Mantém a posição atual por N segundos.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "duration_sec": {"type": "number"},
                    },
                    "required": ["duration_sec"],
                    "additionalProperties": False,
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "land",
                "description": "Comanda modo AUTO.LAND no PX4.",
                "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
            },
        },
        {
            "type": "function",
            "function": {
                "name": "goto_relative",
                "description": "Move o drone uma distância relativa (dx, dy, dz) a partir da posição atual. dx=positivo é frente, dy=positivo é direita, dz=positivo é cima.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "dx": {"type": "number", "description": "Deslocamento em x (frente positivo, metros)"},
                        "dy": {"type": "number", "description": "Deslocamento em y (direita positivo, metros)"},
                        "dz": {"type": "number", "description": "Deslocamento em z (cima positivo, metros)"},
                        "hold_sec": {"type": "number", "description": "Segundos para manter o setpoint (default 4)"},
                    },
                    "required": ["dx", "dy", "dz"],
                    "additionalProperties": False,
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "set_yaw",
                "description": "Define o yaw do drone em graus absolutos. 0=norte, 90=leste, 180=sul, 270=oeste.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "yaw_deg": {"type": "number", "description": "Ângulo de yaw em graus (0-360)"},
                    },
                    "required": ["yaw_deg"],
                    "additionalProperties": False,
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "rotate_yaw",
                "description": "Gira o drone um delta de yaw em graus. Positivo = anti-horário (esquerda), negativo = horário (direita).",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "delta_deg": {"type": "number", "description": "Delta em graus (ex.: 30 para esquerda, -30 para direita)"},
                    },
                    "required": ["delta_deg"],
                    "additionalProperties": False,
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "return_home",
                "description": "Retorna o drone à posição inicial (home) onde estava antes de decolar.",
                "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
            },
        },
        {
            "type": "function",
            "function": {
                "name": "complete_task",
                "description": "Finaliza a missão quando o objetivo foi cumprido ou não é possível continuar.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "summary": {"type": "string", "description": "Resumo do resultado"},
                        "success": {"type": "boolean"},
                    },
                    "required": ["summary", "success"],
                    "additionalProperties": False,
                },
            },
        },
    ]


SYSTEM_PROMPT = """Você é o agente de navegação do drone Oracle Vision (ARNA-style).

Você orquestra ferramentas de percepção e navegação para cumprir tarefas em linguagem natural.
Ambiente: simulação Isaac Sim + PX4 via MAVROS (frame local map, z para cima).

Ferramentas disponíveis:
- get_drone_status: retorna pose, modo, armed, connected
- takeoff(z): decola para altitude z
- goto_position(x,y,z): move para posição absoluta
- goto_relative(dx,dy,dz): move distância relativa (dx=frente, dy=direita, dz=cima)
- set_yaw(yaw_deg): define yaw absoluto em graus
- rotate_yaw(delta_deg): gira delta em graus (positivo=esquerda)
- hover(duration_sec): mantém posição por N segundos
- return_home: retorna à posição onde estava antes de decolar
- land: pouso automático
- get_detections: resumo de detecções de pessoas
- inspect_camera: análise visual da cena
- complete_task(summary, success): finaliza a missão

Regras:
- Sempre comece com get_drone_status.
- Para voar: takeoff antes de goto_position ou goto_relative.
- Para mover: prefira goto_relative para comandos direcionais ("frente", "trás", "esquerda", "direita").
- Para girar: use rotate_yaw (esquerda=positivo, direita=negativo).
- Use get_detections e inspect_camera para tarefas de busca.
- Mantenha movimentos dentro de um raio seguro (~10 m do home).
- Após cumprir a tarefa, chame complete_task.
- Se MAVROS não estiver connected, reporte falha em complete_task.
- Seja conciso; uma ferramenta por vez quando possível.
- A posição home é lembrada pela memória; use return_home para voltar.
"""
