#!/usr/bin/env python3
"""Agrega os eventos de /v2/status gravados no Postgres em métricas de missão.

Calcula por experiment_id:
  - missões iniciadas / concluídas (finished) / concluídas com sucesso
  - taxa de sucesso (success / concluídas)
  - nº de passos até concluir (último vlm_step antes de finished)
  - falhas por tipo (error, no_tool_call, rejected_busy)
  - duração da missão (started -> finished)

Uso:
    python3 metrics.py [--db postgresql://v2:v2@localhost:5432/v2_experiments]
                       [--experiment X]   # filtra um experimento

Saída: tabela Markdown pronta para colar no artigo (alimenta o TODO do abstract).
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from datetime import datetime
from typing import Any

try:
    import psycopg
except ImportError:
    psycopg = None

DB_DEFAULT = "postgresql://v2:v2@localhost:5432/v2_experiments"


def conectar(db_url: str) -> Any:
    if psycopg is None:
        raise SystemExit("psycopg não instalado. Rode: pip install 'psycopg[binary]'")
    return psycopg.connect(db_url)


def carregar_eventos(conn: Any, experiment: str | None) -> list[dict[str, Any]]:
    query = "SELECT experiment_id, ts, event_type, payload FROM experiment_events"
    params: tuple = ()
    if experiment:
        query += " WHERE experiment_id = %s"
        params = (experiment,)
    query += " ORDER BY experiment_id, ts, id"
    with conn.cursor() as cur:
        cur.execute(query, params)
        return [
            {"experiment": r[0], "ts": r[1], "type": r[2], "payload": r[3]}
            for r in cur.fetchall()
        ]


def agrupar_missoes(eventos: list[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    """Agrupa eventos por (experiment, missão). Missão = started..finished."""
    missoes: dict[str, list[dict[str, Any]]] = defaultdict(list)
    atual: list[dict[str, Any]] | None = None

    for ev in eventos:
        t = ev["type"]
        if t == "started":
            atual = [ev]
            missoes[ev["experiment"]].append(atual)  # type: ignore[arg-type]
        elif t in ("finished", "error") and atual is not None:
            atual.append(ev)
            atual = None
        elif atual is not None:
            atual.append(ev)
    return missoes


def metricas_por_experimento(
    missoes: dict[str, list[list[dict[str, Any]]]]
) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for exp, lista in missoes.items():
        m: dict[str, Any] = {
            "iniciadas": len(lista),
            "concluidas": 0,
            "sucesso": 0,
            "falhas": defaultdict(int),
            "passos": [],
            "duracoes_s": [],
        }
        for missao in lista:
            steps = [e for e in missao if e["type"] == "vlm_step"]
            if steps:
                m["passos"].append(int(steps[-1]["payload"].get("step", 0)))
            tipos = [e["type"] for e in missao]
            if "finished" in tipos:
                m["concluidas"] += 1
                fin = next(e for e in missao if e["type"] == "finished")
                if str(fin["payload"].get("success", "false")).lower() == "true":
                    m["sucesso"] += 1
                # duração
                st = next((e for e in missao if e["type"] == "started"), None)
                if st and isinstance(st["ts"], datetime) and isinstance(fin["ts"], datetime):
                    m["duracoes_s"].append((fin["ts"] - st["ts"]).total_seconds())
            for falha in ("error", "no_tool_call", "rejected_busy"):
                if falha in tipos:
                    m["falhas"][falha] += 1
        out[exp] = m
    return out


def fmt_tabela(metricas: dict[str, dict[str, Any]]) -> str:
    linhas = [
        "| Experimento | Missões | Sucesso | Taxa | Passos (med) | Falhas | Duração (s, med) |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    for exp, m in sorted(metricas.items()):
        concluidas = m["concluidas"] or 1
        taxa = m["sucesso"] / concluidas if concluidas else 0.0
        med_p = sum(m["passos"]) / len(m["passos"]) if m["passos"] else "-"
        med_d = sum(m["duracoes_s"]) / len(m["duracoes_s"]) if m["duracoes_s"] else "-"
        falhas = ", ".join(f"{k}={v}" for k, v in m["falhas"].items()) or "-"
        linhas.append(
            f"| {exp} | {m['iniciadas']} | {m['sucesso']} | "
            f"{taxa:.1%} | {med_p} | {falhas} | {med_d} |"
        )
    return "\n".join(linhas)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--db", default=DB_DEFAULT)
    p.add_argument("--experiment", default=None, help="filtra um experiment_id")
    args = p.parse_args()

    conn = conectar(args.db)
    eventos = carregar_eventos(conn, args.experiment)
    if not eventos:
        print("Nenhum evento encontrado no banco.")
        return 1

    missoes = agrupar_missoes(eventos)
    metricas = metricas_por_experimento(missoes)
    print(fmt_tabela(metricas))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
