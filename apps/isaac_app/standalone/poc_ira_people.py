#!/usr/bin/env python
"""POC: pessoas animadas via IRA (isaacsim.replicator.agent), isolada do app.

Valida o pipeline IRA com o mundo mínimo do projeto (assets/ira_ground.usda:
chão em Mesh + PhysicsScene + NavMeshVolume) SEM drone/PX4/MAVROS/câmera/ROS.
Idiom do exemplo oficial tools/actor_sdg/actor_sdg.py do Isaac Sim 6.0.

Uso (HEADLESS=0 abre janela; POC_SECONDS controla a duração):
    ./scripts/poc_ira_people.sh
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

_STANDALONE_DIR = Path(__file__).resolve().parent
_PROJECT_ROOT = _STANDALONE_DIR.parents[2]

from isaacsim import SimulationApp

_HEADLESS = os.getenv("HEADLESS", "0").strip().lower() in {"1", "true", "yes"}
_POC_SECONDS = float(os.getenv("POC_SECONDS", "120"))

simulation_app = SimulationApp(
    {
        "headless": _HEADLESS,
        "extra_args": ["--enable", "isaacsim.replicator.agent.core"],
    }
)

import carb
import omni.timeline
import omni.usd
import yaml
from isaacsim.replicator.agent.core import api as IRA
from isaacsim.replicator.agent.core.configuration.models.root import RootConfig
from pxr import Usd, UsdGeom


def _log(msg: str) -> None:
    print(f"[poc-ira] {msg}", flush=True)


async def _setup() -> bool:
    settings = carb.settings.get_settings()
    settings.set("/app/scripting/ignoreWarningDialog", True)
    settings.set("/app/omni.graph.scriptnode/enable_opt_in", False)
    settings.set("/rtx/raytracing/fractionalCutoutOpacity", True)  # personagens DH
    # Baker de NavMesh: GPU (default) trava indefinidamente em headless sem
    # contexto CUDA; CPU produz navmesh vazia em alguns cenários headless.
    # Default: GPU no GUI (CUDA ativo), CPU no headless. NAVMESH_USE_GPU=0 força CPU.
    _nav_gpu = os.getenv("NAVMESH_USE_GPU", "0" if _HEADLESS else "1").strip().lower() in {"1", "true", "yes"}
    settings.set("/persistent/exts/omni.anim.navigation.core/navMesh/useGpu", _nav_gpu)
    _log(f"navmesh useGpu={_nav_gpu}")

    template = _PROJECT_ROOT / "config" / "ira_people.yaml"
    with open(template) as f:
        data = yaml.safe_load(f)
    ira = data["isaacsim.replicator.agent"]
    ira["environment"]["base_stage_asset_path"] = str(_STANDALONE_DIR / "assets" / "ira_ground.usda")
    group = next(iter(ira["character"]["groups"].values()))
    group["num"] = int(os.getenv("POC_PEOPLE", "10"))

    IRA.set_config(RootConfig.model_validate(data))
    if IRA.get_config_file() is None:
        _log("config IRA inválida")
        return False

    await IRA.setup_simulation()
    return True


def _character_positions() -> dict[str, tuple[float, float, float]]:
    """Posições mundo dos personagens sob /World/Characters (prova de movimento)."""
    stage = omni.usd.get_context().get_stage()
    cache = UsdGeom.XformCache()
    out: dict[str, tuple[float, float, float]] = {}
    chars = stage.GetPrimAtPath("/World/Characters")
    if not chars or not chars.IsValid():
        return out
    for child in chars.GetAllChildren():
        try:
            t = cache.GetLocalToWorldTransform(child).ExtractTranslation()
            out[str(child.GetPath())] = (round(float(t[0]), 2), round(float(t[1]), 2), round(float(t[2]), 2))
        except Exception:
            continue
    return out


def main() -> None:
    from omni.kit.async_engine import run_coroutine

    task = run_coroutine(_setup())
    while not task.done():
        simulation_app.update()
    if task.exception() is not None:
        _log(f"setup_simulation falhou: {task.exception()!r}")
        simulation_app.close()
        sys.exit(1)

    n = len(_character_positions())
    _log(f"setup OK: {n} personagens no stage; dando play por {_POC_SECONDS:.0f}s")
    if n == 0:
        _log("NENHUM personagem carregado — investigar navmesh/character loader")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    t0 = time.monotonic()
    last_report = 0.0
    initial = _character_positions()
    while simulation_app.is_running() and (time.monotonic() - t0) < _POC_SECONDS:
        simulation_app.update()
        now = time.monotonic() - t0
        if now - last_report >= 5.0:
            last_report = now
            pos = _character_positions()
            moved = sum(
                1
                for path, p in pos.items()
                if path in initial and (abs(p[0] - initial[path][0]) + abs(p[1] - initial[path][1])) > 0.3
            )
            sample = next(iter(pos.items()), None)
            _log(
                f"t={now:5.1f}s | {len(pos)} personagens | {moved} em movimento"
                + (f" | ex: {sample[0].split('/')[-1]} em {sample[1]}" if sample else "")
            )

    _log("encerrando POC")
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
