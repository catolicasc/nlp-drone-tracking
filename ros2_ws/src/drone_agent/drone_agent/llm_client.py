"""Cliente HTTP OpenAI-compatible para chat + function calling."""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from typing import Any


class LlmClientError(RuntimeError):
    pass


class LlmClient:
    def __init__(
        self,
        api_base: str,
        api_key: str,
        model: str,
        timeout_sec: float = 120.0,
    ) -> None:
        self.api_base = api_base.rstrip("/")
        self.api_key = api_key
        self.model = model
        self.timeout_sec = timeout_sec

    @classmethod
    def from_env(
        cls,
        api_base: str | None = None,
        model: str | None = None,
        timeout_sec: float = 120.0,
    ) -> LlmClient:
        key = (
            os.environ.get("LVLM_API_KEY")
            or os.environ.get("OPENAI_API_KEY")
            or ""
        ).strip()
        if not key:
            raise LlmClientError(
                "Defina LVLM_API_KEY ou OPENAI_API_KEY no .env"
            )
        base = (api_base or os.environ.get("LVLM_API_BASE") or "https://api.openai.com/v1").strip()
        mdl = (model or os.environ.get("LVLM_MODEL") or "gpt-4o-mini").strip()
        return cls(base, key, mdl, timeout_sec)

    def chat(
        self,
        messages: list[dict[str, Any]],
        tools: list[dict[str, Any]] | None = None,
        temperature: float = 0.2,
    ) -> dict[str, Any]:
        body: dict[str, Any] = {
            "model": self.model,
            "messages": messages,
            "temperature": temperature,
        }
        if tools:
            body["tools"] = tools
            body["tool_choice"] = "auto"

        req = urllib.request.Request(
            f"{self.api_base}/chat/completions",
            data=json.dumps(body).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.api_key}",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_sec) as resp:
                payload = json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise LlmClientError(f"LLM HTTP {exc.code}: {detail}") from exc
        except urllib.error.URLError as exc:
            raise LlmClientError(f"LLM request failed: {exc}") from exc

        if "error" in payload:
            raise LlmClientError(str(payload["error"]))
        return payload

    @staticmethod
    def assistant_message(response: dict[str, Any]) -> dict[str, Any]:
        try:
            return response["choices"][0]["message"]
        except (KeyError, IndexError) as exc:
            raise LlmClientError(f"Resposta LLM inválida: {response}") from exc
