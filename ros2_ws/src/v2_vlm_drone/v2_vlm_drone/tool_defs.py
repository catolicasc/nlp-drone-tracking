"""Tool schemas and system prompt for Oracle Vision V2."""

from __future__ import annotations

from typing import Any


SYSTEM_PROMPT = """Você é o agente VLM-only do drone Oracle Vision V2.

Você recebe tarefas em linguagem natural e controla o drone por ferramentas.
Ambiente: Isaac Sim + Pegasus + PX4 SITL via MAVROS. Frame local: x para frente,
y lateral, z para cima. Você nunca inventa estado: consulte ferramentas.

Regras de segurança:
- Sempre comece com get_drone_status.
- Só use takeoff/goto/rotate se connected=true.
- Use takeoff antes de movimentos se o drone estiver desarmado.
- Respeite altitude e raio máximos retornados por get_drone_status.
- land é sempre permitido.
- Para tarefas visuais, use inspect_camera.
- Termine toda tarefa com complete_task.
"""


def tool_schemas() -> list[dict[str, Any]]:
    return [
        {
            "type": "function",
            "function": {
                "name": "get_drone_status",
                "description": "Retorna conexão MAVROS, modo PX4, armed, pose, setpoint e limites de segurança.",
                "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
            },
        },
        {
            "type": "function",
            "function": {
                "name": "inspect_camera",
                "description": "Analisa a imagem atual da câmera do drone com o VLM multimodal.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "question": {
                            "type": "string",
                            "description": "Pergunta objetiva sobre o frame atual da câmera.",
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
                "description": "Entra em OFFBOARD, arma e sobe para altitude z em metros.",
                "parameters": {
                    "type": "object",
                    "properties": {"z": {"type": "number", "description": "Altitude alvo em metros."}},
                    "required": ["z"],
                    "additionalProperties": False,
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "goto_position",
                "description": "Move para posição local absoluta x,y,z em metros.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "z": {"type": "number"},
                        "hold_sec": {"type": "number", "description": "Tempo para manter o setpoint."},
                    },
                    "required": ["x", "y", "z"],
                    "additionalProperties": False,
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "goto_relative",
                "description": "Move relativo à pose atual, em metros no frame local.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "dx": {"type": "number"},
                        "dy": {"type": "number"},
                        "dz": {"type": "number"},
                        "hold_sec": {"type": "number"},
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
                "description": "Define yaw absoluto em graus.",
                "parameters": {
                    "type": "object",
                    "properties": {"yaw_deg": {"type": "number"}},
                    "required": ["yaw_deg"],
                    "additionalProperties": False,
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "rotate_yaw",
                "description": "Gira yaw relativo em graus.",
                "parameters": {
                    "type": "object",
                    "properties": {"delta_deg": {"type": "number"}},
                    "required": ["delta_deg"],
                    "additionalProperties": False,
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "hover",
                "description": "Mantém o setpoint atual por duration_sec segundos.",
                "parameters": {
                    "type": "object",
                    "properties": {"duration_sec": {"type": "number"}},
                    "required": ["duration_sec"],
                    "additionalProperties": False,
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "land",
                "description": "Comanda AUTO.LAND e para setpoints conflitantes.",
                "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
            },
        },
        {
            "type": "function",
            "function": {
                "name": "complete_task",
                "description": "Finaliza a tarefa com resumo e sucesso/falha.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "summary": {"type": "string"},
                        "success": {"type": "boolean"},
                    },
                    "required": ["summary", "success"],
                    "additionalProperties": False,
                },
            },
        },
    ]
