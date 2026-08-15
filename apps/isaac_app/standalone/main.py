#!/usr/bin/env python
"""
Oracle Vision — standalone Isaac Sim app (padrão Pegasus, Isaac Sim 6.0).

Bootstrap via SimulationApp; lógica de simulação na classe OracleVisionApp.

Isaac Sim 6.0: a classe World foi removida → SimulationManager/RenderingManager
com callbacks de simulação/render; o loop usa simulation_app.update().
"""

from __future__ import annotations

import math
import os
import random
import sys
import threading
import time
from pathlib import Path

_STANDALONE_DIR = Path(__file__).resolve().parent
if str(_STANDALONE_DIR) not in sys.path:
    sys.path.insert(0, str(_STANDALONE_DIR))

_pegasus_path = os.getenv("PEGASUS_PATH")
if _pegasus_path:
    _pegasus_paths = [
        Path(_pegasus_path),
        Path(_pegasus_path) / "extensions" / "pegasus.simulator",
    ]
    for _path in _pegasus_paths:
        if _path.exists() and str(_path) not in sys.path:
            sys.path.insert(0, str(_path))

import carb
from isaacsim import SimulationApp
from config_loader import load_config as _load_config

# O config precisa ser lido antes do SimulationApp para decidir se a extensão
# do IRA (pessoas animadas) entra no bootstrap via --enable.
try:
    _bootstrap_config, _bootstrap_root = _load_config()
    _PEOPLE_ANIMATED = bool((_bootstrap_config.get("people", {}) or {}).get("animated", False))
except Exception:
    _bootstrap_config = {}
    _bootstrap_root = Path.cwd()
    _PEOPLE_ANIMATED = False

# Só é True se o setup do IRA de fato concluiu (ver _setup_ira_people); o
# _setup_people usa isso para decidir entre pessoas IRA e spawn estático.
_IRA_PEOPLE_LOADED = False

_extra_args = ["--enable", "isaacsim.ros2.bridge"]
if _PEOPLE_ANIMATED:
    _extra_args += ["--enable", "isaacsim.replicator.agent.core"]

_HEADLESS = os.getenv("HEADLESS", "0").strip().lower() in {"1", "true", "yes"}
simulation_app = SimulationApp(
    {
        "headless": _HEADLESS,
        "extra_args": _extra_args,
    }
)

import omni.timeline
import omni.usd
from isaacsim.core.rendering_manager import RenderingManager, RenderingEvent
from isaacsim.core.simulation_manager import SimulationManager, SimulationEvent
import isaacsim.core.experimental.utils.stage as _stage_utils
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

from config_loader import load_config, resolve_usd_path
from sensors import setup_sensors
from spawn import PegasusQuadrotorSpec, PersonSpawnSpec, spawn_pegasus_quadrotor, spawn_people


def _get_stage():
    """Stage USD atual (substitui omni.isaac.core.utils.stage.get_current_stage)."""
    return omni.usd.get_context().get_stage()


def _is_prim_path_valid(prim_path: str) -> bool:
    """True se o prim existe no stage (substitui omni.isaac.core.utils.prims.is_prim_path_valid)."""
    if not prim_path:
        return False
    prim = _get_stage().GetPrimAtPath(prim_path)
    return prim is not None and prim.IsValid()


def _add_reference_to_stage(usd_path: str, prim_path: str) -> None:
    """Referencia um USD no stage (substitui omni.isaac.core.utils.stage.add_reference_to_stage)."""
    _stage_utils.add_reference_to_stage(usd_path=usd_path, path=prim_path)


def _add_default_ground_plane(size: float = 250.0) -> None:
    """Ground plane com colisão via USD puro (padrão do exemplo 0_template do Pegasus 6.0).

    Cubo fino com topo em z=0 (superfície do chão).
    """
    stage = _get_stage()
    ground = UsdGeom.Cube.Define(stage, "/World/DefaultGroundPlane")
    ground.GetSizeAttr().Set(size)
    xform = UsdGeom.Xformable(ground.GetPrim())
    thickness = size * 0.02
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.5 * thickness))
    xform.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, 0.02))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())


class PersonFoundListener:
    """Subscriber ROS 2 em thread separada (não bloqueia o loop de simulação)."""

    def __init__(self, topic_name: str = "/person_found"):
        self._topic_name = topic_name
        self._lock = threading.Lock()
        self._person_found = False
        self._started = False

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        threading.Thread(target=self._spin, daemon=True).start()

    def get(self) -> bool:
        with self._lock:
            return bool(self._person_found)

    def _set(self, val: bool) -> None:
        with self._lock:
            self._person_found = bool(val)

    def _spin(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import Bool

            try:
                rclpy.init(args=None)
            except Exception:
                pass

            class _Node(Node):
                def __init__(self, outer: PersonFoundListener):
                    super().__init__("isaac_person_found_listener")

                    def cb(msg: Bool) -> None:
                        outer._set(bool(msg.data))

                    self.create_subscription(Bool, outer._topic_name, cb, 10)

            rclpy.spin(_Node(self))
        except Exception as exc:  # noqa: BLE001
            print(f"Falha ao iniciar subscriber ROS2 ({self._topic_name}): {exc}")


class OracleVisionApp:
    """App standalone: stage + cena + drone Pegasus/PX4 + sensores + callbacks."""

    def __init__(self) -> None:
        self.config, self.project_root = load_config()
        self.timeline = omni.timeline.get_timeline_interface()
        self.stop_sim = False

        self._light = None
        self._person_found_listener: PersonFoundListener | None = None
        self._last_blink_t = time.monotonic()
        self._blink_on = False

        self._print_banner()

        self._setup_world()
        self._setup_people()
        self._setup_drone_and_sensors()
        self._register_callbacks()

        print("Cena pronta.")

    def _print_banner(self) -> None:
        print("=" * 60)
        print("Oracle Vision — Isaac Sim")
        print(f"Root do projeto: {self.project_root}")
        print(f"Config carregada: {self.config}")
        print("=" * 60)
        print(f"ISAAC_SIM_PATH={os.getenv('ISAAC_SIM_PATH')}")
        print(f"PEGASUS_PATH={os.getenv('PEGASUS_PATH')}")
        print(f"PX4_PATH={os.getenv('PX4_PATH')}")

    def _setup_world(self) -> None:
        # No modo animado o IRA abre o assets/ira_ground.usda como stage raiz
        # (chão 40 m + luz): não recriar/sobrescrever o que já existe.
        if not _is_prim_path_valid("/World/DefaultGroundPlane"):
            _add_default_ground_plane()
        self._load_usd_scene()
        self._load_isaac_environment()
        if not _is_prim_path_valid("/World/Light/DomeLight"):
            self._setup_lighting()

    def _setup_lighting(self) -> None:
        # Sem nenhuma luz na stage o RTX renderiza tudo preto (câmera e viewport);
        # dome light branca no lugar da HDR do S3 para não depender de streaming.
        stage = _get_stage()
        light = UsdLux.DomeLight.Define(stage, "/World/Light/DomeLight")
        light.CreateIntensityAttr(5e3)
        light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))

    def _load_usd_scene(self) -> None:
        usd_cfg = self.config.get("world", {}).get("usd")
        if not usd_cfg:
            return

        usd_full_path = resolve_usd_path(self.project_root, usd_cfg)
        print(f"Tentando carregar USD: {usd_full_path}")

        if not os.path.exists(usd_full_path):
            print("USD não encontrado, seguindo só com o ground plane.")
            return

        prim_path = "/World/Environment"
        if not _is_prim_path_valid(prim_path):
            _add_reference_to_stage(usd_path=usd_full_path, prim_path=prim_path)
            print(f"USD carregado em {prim_path}")

    def _load_isaac_environment(self) -> None:
        world_asset_rel = self.config.get("world", {}).get("isaac_asset_rel_path")
        if not world_asset_rel:
            return

        assets_root = get_assets_root_path()
        if not assets_root:
            print("Não consegui localizar os assets do Isaac Sim.")
            return

        usd_path = f"{assets_root}/Isaac/{str(world_asset_rel).lstrip('/')}"
        prim_path = "/World/Environment"
        if not _is_prim_path_valid(prim_path):
            _add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            print(f"Ambiente Isaac carregado em {prim_path}: {usd_path}")

    def _spawn_bounds_from_default_prim(self) -> tuple[tuple[float, float], float] | None:
        stage = _get_stage()
        default_prim = stage.GetDefaultPrim()
        if not default_prim or not default_prim.IsValid():
            return None

        try:
            bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
            bbox = bbox_cache.ComputeWorldBound(default_prim)
            r = bbox.GetRange()
            minp, maxp = r.GetMin(), r.GetMax()
            if not (minp and maxp):
                return None

            minx, miny = float(minp[0]), float(minp[1])
            maxx, maxy = float(maxp[0]), float(maxp[1])
            if not all(math.isfinite(v) for v in (minx, miny, maxx, maxy)):
                return None

            cx = 0.5 * (minx + maxx)
            cy = 0.5 * (miny + maxy)
            half = 0.5 * min(maxx - minx, maxy - miny)
            if half <= 0.1:
                return None
            return (cx, cy), float(half * 0.9)
        except Exception:
            return None

    def _setup_people(self) -> None:
        if _IRA_PEOPLE_LOADED:
            print("Pessoas animadas (IRA) já carregadas no setup do stage; spawn estático ignorado.")
            return

        people_cfg = self.config.get("people", {}) or {}
        n_people = int(people_cfg.get("count", 20))
        if n_people <= 0:
            return

        min_distance_m = float(people_cfg.get("min_distance_m", 10.0))
        people_z = float(people_cfg.get("z", 0.0))
        world_cfg = self.config.get("world", {}) or {}
        area_half = float(world_cfg.get("spawn_area_half_extent_m", 60.0))
        area_center = world_cfg.get("spawn_area_center_xy", [0.0, 0.0])
        try:
            area_center_xy = (float(area_center[0]), float(area_center[1]))
        except Exception:
            area_center_xy = (0.0, 0.0)

        if bool(world_cfg.get("spawn_area_from_default_prim", False)):
            auto_bounds = self._spawn_bounds_from_default_prim()
            if auto_bounds is not None:
                area_center_xy, area_half = auto_bounds
                print(
                    "[people] auto bounds from defaultPrim: "
                    f"center=({area_center_xy[0]:.2f},{area_center_xy[1]:.2f}) half={area_half:.2f}"
                )

        self._spawn_random_people(
            n_people=n_people,
            min_distance_m=min_distance_m,
            area_half_extent_m=area_half,
            area_center_xy=area_center_xy,
            z=people_z,
        )

    def _spawn_random_people(
        self,
        n_people: int,
        min_distance_m: float,
        area_half_extent_m: float,
        area_center_xy: tuple[float, float],
        z: float,
    ) -> None:
        assets_root = get_assets_root_path()
        if not assets_root:
            print("Não consegui localizar os assets do Isaac Sim.")
            return

        # Asset do personagem no Isaac 6.0 (em 5.1 era original_male_adult_police_04).
        asset_rel_path = "People/Characters/male_adult_police_04/male_adult_police_04.usd"
        min_d_requested = max(0.0, float(min_distance_m))
        half = max(1.0, float(area_half_extent_m))
        cx, cy = float(area_center_xy[0]), float(area_center_xy[1])
        z = float(z)

        print(
            "[people] spawn bounds: "
            f"x=[{(cx - half):.2f},{(cx + half):.2f}] "
            f"y=[{(cy - half):.2f},{(cy + half):.2f}] "
            f"min_distance={min_d_requested:.2f}m count={int(n_people)}"
        )

        def sample_positions(min_d: float) -> list[tuple[float, float, float]]:
            positions: list[tuple[float, float, float]] = []
            max_attempts = max(4000, n_people * 800)
            attempts = 0
            while len(positions) < n_people and attempts < max_attempts:
                attempts += 1
                x = random.uniform(cx - half, cx + half)
                y = random.uniform(cy - half, cy + half)
                if min_d > 0.0 and any(math.hypot(x - px, y - py) < min_d for px, py, _ in positions):
                    continue
                positions.append((float(x), float(y), float(z)))
            return positions

        min_d = float(min_d_requested)
        positions = sample_positions(min_d)
        while len(positions) < n_people and min_d > 0.0:
            min_d = max(0.0, min_d * 0.9)
            positions = sample_positions(min_d)
        if len(positions) < n_people:
            positions = sample_positions(0.0)

        specs = [
            PersonSpawnSpec(
                prim_path=f"/World/People/person_{i}",
                asset_rel_path=asset_rel_path,
                position=p,
            )
            for i, p in enumerate(positions, start=1)
        ]
        n = spawn_people(assets_root, specs)
        if min_d_requested > 0.0 and min_d < min_d_requested:
            print(f"{n} pessoas adicionadas. (min_distance relaxada: {min_d_requested:.2f} -> {min_d:.2f} m)")
        else:
            print(f"{n} pessoas adicionadas.")

    def _setup_drone_and_sensors(self) -> None:
        px4_path = os.getenv("PX4_PATH")
        if not px4_path:
            print("PX4_PATH não definido; drone não foi criado.")
            return

        drones_cfg = self.config.get("drones", []) or []
        drone_pos = (0.0, 0.0, 0.0)
        if drones_cfg:
            pos = drones_cfg[0].get("position", drone_pos)
            try:
                drone_pos = (float(pos[0]), float(pos[1]), float(pos[2]))
            except Exception:
                pass

        spawn_pegasus_quadrotor(
            px4_path,
            PegasusQuadrotorSpec(init_pos=drone_pos),
        )
        print("Drone adicionado com sucesso.")
        setup_sensors(self.config, "/World/quadrotor")
        self._light = self._ensure_drone_light("/World/quadrotor")
        self._person_found_listener = PersonFoundListener("/person_found")
        self._person_found_listener.start()

    def _ensure_drone_light(self, drone_prim_path: str):
        stage = _get_stage()
        light_path = f"{drone_prim_path}/person_light"
        light = UsdLux.SphereLight.Define(stage, light_path)
        light.CreateRadiusAttr(0.05)
        light.CreateIntensityAttr(0.0)
        light.CreateColorAttr((1.0, 0.2, 0.2))
        return light

    def _register_callbacks(self) -> None:
        self._cb_render = RenderingManager.register_callback(
            RenderingEvent.NEW_FRAME, callback=self._render_callback
        )
        self._cb_stop = SimulationManager.register_callback(
            self._on_sim_stopped, SimulationEvent.SIMULATION_STOPPED
        )

    def _render_callback(self, event) -> None:
        if self._light is None or self._person_found_listener is None:
            return

        found = self._person_found_listener.get()
        if found:
            now = time.monotonic()
            if now - self._last_blink_t >= 0.25:
                self._blink_on = not self._blink_on
                self._last_blink_t = now
            intensity = 2000.0 if self._blink_on else 0.0
        else:
            intensity = 0.0
            self._blink_on = False
            self._last_blink_t = time.monotonic()

        try:
            self._light.GetIntensityAttr().Set(float(intensity))
        except Exception:
            pass

    def _on_sim_stopped(self, *args) -> None:
        self.stop_sim = True

    def run(self) -> None:
        self.stop_sim = False
        self.timeline.play()

        while simulation_app.is_running() and not self.stop_sim:
            simulation_app.update()

        carb.log_warn("Oracle Vision simulation app is closing.")
        SimulationManager.deregister_callback(self._cb_stop)
        RenderingManager.deregister_callback(self._cb_render)
        self.timeline.stop()
        simulation_app.close()


def _setup_ira_people(config: dict, project_root) -> bool:
    """Soba o pipeline IRA de pessoas animadas como dono do stage raiz.

    Idiom do exemplo oficial tools/actor_sdg/actor_sdg.py: o IRA abre o
    environment (nosso assets/ira_ground.usda) como camada raiz, assa o NavMesh
    e spawna os personagens com rotina wander. O drone Pegasus e a câmera são
    adicionados depois, em cima do stage aberto pelo IRA. A extensão precisa
    ter sido habilitada no bootstrap (ver extra_args do SimulationApp).
    """
    try:
        import yaml as _yaml

        import carb as _carb
        from isaacsim.replicator.agent.core import api as IRA
        from isaacsim.replicator.agent.core.configuration.models.root import RootConfig
    except Exception as e:  # noqa: BLE001
        print(f"[ira] extensão isaacsim.replicator.agent.core indisponível ({e}); uso pessoas estáticas")
        return False

    settings = _carb.settings.get_settings()
    settings.set("/app/scripting/ignoreWarningDialog", True)
    settings.set("/app/omni.graph.scriptnode/enable_opt_in", False)
    settings.set("/rtx/raytracing/fractionalCutoutOpacity", True)  # personagens DH
    # Baker de NavMesh: GPU (default) trava em headless sem CUDA; CPU pode
    # produzir navmesh vazia em alguns cenários. Default: GPU no GUI, CPU no
    # headless. NAVMESH_USE_GPU=0 força CPU.
    _nav_gpu = os.getenv("NAVMESH_USE_GPU", "0" if _HEADLESS else "1").strip().lower() in {"1", "true", "yes"}
    settings.set("/persistent/exts/omni.anim.navigation.core/navMesh/useGpu", _nav_gpu)

    people_cfg = config.get("people", {}) or {}
    template_path = Path(project_root) / str(people_cfg.get("ira_config", "config/ira_people.yaml"))
    try:
        with open(template_path) as f:
            data = _yaml.safe_load(f)
    except Exception as e:  # noqa: BLE001
        print(f"[ira] template {template_path} ilegível ({e}); uso pessoas estáticas")
        return False

    ira = data["isaacsim.replicator.agent"]
    ira["environment"]["base_stage_asset_path"] = str(
        Path(__file__).resolve().parent / "assets" / "ira_ground.usda"
    )
    group = next(iter(ira["character"]["groups"].values()))
    group["num"] = int(people_cfg.get("count", 10))

    IRA.set_config(RootConfig.model_validate(data))
    if IRA.get_config_file() is None:
        print("[ira] config inválida; uso pessoas estáticas")
        return False

    async def _run():
        await IRA.setup_simulation()

    from omni.kit.async_engine import run_coroutine

    task = run_coroutine(_run())
    while not task.done():
        simulation_app.update()
    if task.exception() is not None:
        print(f"[ira] setup_simulation falhou: {task.exception()!r}; uso pessoas estáticas")
        return False

    print(f"[ira] {group['num']} pessoas animadas (wander) carregadas")
    global _IRA_PEOPLE_LOADED
    _IRA_PEOPLE_LOADED = True
    return True


def main() -> None:
    if _PEOPLE_ANIMATED:
        # Ordem obrigatória: IRA primeiro (abre o stage raiz); drone/câmera depois.
        _setup_ira_people(_bootstrap_config, _bootstrap_root)
    OracleVisionApp().run()


if __name__ == "__main__":
    main()
