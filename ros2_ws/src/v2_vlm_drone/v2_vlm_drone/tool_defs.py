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
- Para tarefas visuais simples, use inspect_camera.
- Para detectar pessoa no frame atual, use detect_person_in_frame (YOLO local + confirmação VLM opcional).
- Para procurar pessoas girando o drone, use scan_for_people depois de decolar.
- Termine toda tarefa com complete_task.

Regras de busca de alvo (CRÍTICO):
- Quando a tarefa descrever um alvo específico (ex.: "mulher, cabelo loiro, camisa azul"),
  EXTRAIA essas características e passe em search_for_target(target_description="...").
  A descrição fica guardada na memória da missão e o loop de busca só termina quando
  o VLM confirmar o alvo correspondente.
- land (pouso normal) SÓ após get_drone_status ou search_for_target retornarem
  target_found=true. O pouso normal é consequência da confirmação do alvo,
  NÃO a próxima etapa linear após uma tentativa de busca.
- Se search_for_target retornar found=false com abort_reason (timeout, bateria,
  conexão perdida, varredura esgotada), você pode: tentar nova busca com padrão
  diferente, mover para novo setor, ou encerrar com safe_abort.
- Em qualquer condição crítica (bateria baixa, conexão perdida, comportamento
  anômalo), use safe_abort imediatamente — ele executa RTL com fallback LAND.
"""


def json_tool_protocol_prompt() -> str:
    tools = [
        {
            "name": item["function"]["name"],
            "description": item["function"]["description"],
            "parameters": item["function"]["parameters"],
        }
        for item in tool_schemas()
    ]
    return (
        "O servidor local não suporta OpenAI tools nativas. Mesmo assim, você deve escolher tools.\n"
        "Responda somente JSON válido, sem markdown e sem texto extra, em um destes formatos:\n"
        '{"tool": "nome_da_tool", "arguments": {...}}\n'
        'ou {"tool_calls": [{"tool": "nome_da_tool", "arguments": {...}}]}.\n'
        "Use exatamente uma ou mais tools da lista abaixo. Quando terminar, chame complete_task.\n\n"
        f"Tools disponíveis:\n{tools}"
    )


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
                "name": "detect_person_in_frame",
                "description": (
                    "Detecta pessoas no frame atual com YOLO. Se houver candidato, pode confirmar com VLM "
                    "e salvar evidência em runs/search_evidence."
                ),
                "parameters": {
                    "type": "object",
                    "properties": {
                        "confirm_vlm": {
                            "type": "boolean",
                            "description": "Se true, confirma candidatos YOLO com VLM antes de marcar found.",
                        }
                    },
                    "additionalProperties": False,
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "scan_for_people",
                "description": (
                    "Procura pessoas girando o drone em etapas. Usa YOLO em cada frame e confirma com VLM "
                    "somente quando houver candidato. Retorna found=true quando uma pessoa for confirmada."
                ),
                "parameters": {
                    "type": "object",
                    "properties": {
                        "sweeps": {
                            "type": "integer",
                            "description": "Número de voltas completas de varredura.",
                        },
                        "step_deg": {
                            "type": "number",
                            "description": "Graus por etapa de yaw. Use 45 ou 60 para busca rápida.",
                        },
                        "settle_sec": {
                            "type": "number",
                            "description": "Tempo de espera após cada giro antes de analisar a câmera.",
                        },
                        "stop_on_found": {
                            "type": "boolean",
                            "description": "Se true, encerra a varredura assim que encontrar pessoa.",
                        },
                        "confirm_vlm": {
                            "type": "boolean",
                            "description": "Se true, confirma detecções YOLO com VLM.",
                        },
                    },
                    "additionalProperties": False,
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "search_for_target",
                "description": (
                    "Busca em LOOP até confirmar o alvo descrito: varia o yaw, roda YOLO por frame "
                    "e, diante de candidato, pede confirmação ao VLM contra target_description. "
                    "Sai por confirmação (found=true), timeout de busca, bateria mínima, perda de "
                    "conexão ou varredura esgotada (nestes casos found=false com abort_reason e "
                    "você deve chamar safe_abort). Use após takeoff."
                ),
                "parameters": {
                    "type": "object",
                    "properties": {
                        "target_description": {
                            "type": "string",
                            "description": (
                                "Características do alvo extraídas da tarefa, ex.: "
                                "'mulher, cabelo loiro, camisa azul'. Obrigatório."
                            ),
                        },
                        "sweeps": {
                            "type": "integer",
                            "description": "Número máximo de voltas completas de varredura (1-6).",
                        },
                        "step_deg": {
                            "type": "number",
                            "description": "Graus por etapa de yaw (15-120; use 45 ou 60).",
                        },
                        "settle_sec": {
                            "type": "number",
                            "description": "Tempo de estabilização após cada giro antes de analisar.",
                        },
                        "confirm_vlm": {
                            "type": "boolean",
                            "description": "Se true (recomendado), confirma candidatos YOLO contra a descrição via VLM.",
                        },
                    },
                    "required": ["target_description"],
                    "additionalProperties": False,
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "safe_abort",
                "description": (
                    "Aborto por condição de segurança: para setpoints, executa AUTO.RTL com "
                    "fallback AUTO.LAND e encerra a missão como falha controlada. Use quando a "
                    "busca retornar abort_reason (timeout/bateria/conexão) ou em qualquer anomalia crítica."
                ),
                "parameters": {
                    "type": "object",
                    "properties": {
                        "reason": {
                            "type": "string",
                            "description": "Motivo do aborto (ex.: 'bateria crítica', 'timeout de busca').",
                        }
                    },
                    "required": ["reason"],
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
                "description": (
                    "Pouso normal (AUTO.LAND). Bloqueado enquanto target_found=false — só use "
                    "após search_for_target confirmar o alvo. Em emergência, prefira safe_abort "
                    "(ou force=true)."
                ),
                "parameters": {
                    "type": "object",
                    "properties": {
                        "force": {
                            "type": "boolean",
                            "description": "Se true, pousa mesmo sem alvo confirmado (emergência).",
                        }
                    },
                    "additionalProperties": False,
                },
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
