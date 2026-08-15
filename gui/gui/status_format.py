"""Formatação de eventos /v2/status — sem dependência de Qt (testável isoladamente)."""

from __future__ import annotations

from typing import Any

# cor por evento de /v2/status
EVENT_COLORS = {
    "ready": "#7f8c8d",
    "started": "#2980b9",
    "vlm_step": "#3498db",
    "tool_result": "#27ae60",
    "finished": "#8e44ad",
    "interrupt_land": "#e67e22",
    "rejected_busy": "#f39c12",
    "no_tool_call": "#e67e22",
    "error": "#e74c3c",
}


def format_status_event(payload: dict[str, Any]) -> str:
    """Resume um evento de /v2/status em uma linha legível."""
    event = payload.get("event", "desconhecido")
    if event == "vlm_step":
        return f"passo VLM {payload.get('step')}/{payload.get('max_steps')}"
    if event == "tool_result":
        result = str(payload.get("result", ""))[:160]
        return f"tool {payload.get('tool')}: {result}"
    if event == "started":
        return f"tarefa iniciada: {payload.get('task', '')}"
    if event == "ready":
        return f"agente pronto (modelo {payload.get('model', '?')})"
    if event == "finished":
        return f"concluída: {payload.get('summary', payload.get('result', ''))}"
    if event == "error":
        return f"erro: {payload.get('detail', payload.get('error', payload))}"
    return event + (
        f" — {payload.get('task') or payload.get('content') or payload.get('hint', '')}"
        if any(k in payload for k in ("task", "content", "hint"))
        else ""
    )
