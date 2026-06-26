"""OpenAI-compatible VLM client for Oracle Vision V2."""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any


class VlmClientError(RuntimeError):
    pass


@dataclass(frozen=True)
class VlmConfig:
    api_base: str
    api_key: str
    model: str
    timeout_sec: float = 120.0


class VlmClient:
    def __init__(self, config: VlmConfig) -> None:
        self.config = config

    @classmethod
    def from_env(cls) -> VlmClient:
        api_key = (
            os.environ.get("LVLM_API_KEY")
            or os.environ.get("OPENAI_API_KEY")
            or ""
        ).strip()
        if not api_key:
            raise VlmClientError(
                "LVLM_API_KEY ou OPENAI_API_KEY não configurado."
            )

        api_base = (
            os.environ.get("LVLM_API_BASE")
            or "https://api.openai.com/v1"
        ).strip()
        model = (
            os.environ.get("LVLM_MODEL")
            or "gpt-4o-mini"
        ).strip()
        timeout = 120.0
        return cls(VlmConfig(api_base=api_base.rstrip("/"), api_key=api_key, model=model, timeout_sec=timeout))

    @property
    def model(self) -> str:
        return self.config.model

    @property
    def api_base(self) -> str:
        return self.config.api_base

    def chat(
        self,
        messages: list[dict[str, Any]],
        *,
        tools: list[dict[str, Any]] | None = None,
        temperature: float = 0.2,
    ) -> dict[str, Any]:
        body: dict[str, Any] = {
            "model": self.config.model,
            "messages": messages,
            "temperature": temperature,
        }
        if tools:
            body["tools"] = tools
            body["tool_choice"] = "auto"

        req = urllib.request.Request(
            f"{self.config.api_base}/chat/completions",
            data=json.dumps(body).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.config.api_key}",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self.config.timeout_sec) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise VlmClientError(f"VLM HTTP {exc.code}: {detail}") from exc
        except urllib.error.URLError as exc:
            raise VlmClientError(f"VLM request failed: {exc}") from exc

    @staticmethod
    def assistant_message(response: dict[str, Any]) -> dict[str, Any]:
        try:
            return response["choices"][0]["message"]
        except (KeyError, IndexError) as exc:
            raise VlmClientError(f"Resposta VLM inválida: {response}") from exc
