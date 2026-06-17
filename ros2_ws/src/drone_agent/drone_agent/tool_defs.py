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

Regras:
- Sempre comece com get_drone_status.
- Para voar: takeoff antes de goto_position.
- Use get_detections e inspect_camera para tarefas de busca.
- Mantenha movimentos dentro de um raio seguro (~10 m do home).
- Após cumprir a tarefa, chame complete_task.
- Se MAVROS não estiver connected, reporte falha em complete_task.
- Seja conciso; uma ferramenta por vez quando possível.
"""
