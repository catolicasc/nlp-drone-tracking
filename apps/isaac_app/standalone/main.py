import os
import sys
from pathlib import Path
import random
import math
import threading
import time

_STANDALONE_DIR = Path(__file__).resolve().parent
if str(_STANDALONE_DIR) not in sys.path:
    sys.path.insert(0, str(_STANDALONE_DIR))

import omni.kit.app
from omni.isaac.core import World
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.nucleus import get_assets_root_path
from omni.kit.async_engine import run_coroutine
from pxr import Gf, Usd, UsdGeom, UsdLux

from config_loader import load_config, resolve_usd_path
from sensors import setup_sensors
from spawn import PegasusQuadrotorSpec, PersonSpawnSpec, spawn_pegasus_quadrotor, spawn_people


class _PersonFoundListener:
    def __init__(self, topic_name: str = "/person_found"):
        self._topic_name = topic_name
        self._lock = threading.Lock()
        self._person_found = False
        self._started = False

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        t = threading.Thread(target=self._spin, daemon=True)
        t.start()

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
                def __init__(self, outer: "_PersonFoundListener"):
                    super().__init__("isaac_person_found_listener")

                    def cb(msg: Bool) -> None:
                        outer._set(bool(msg.data))

                    self.create_subscription(Bool, outer._topic_name, cb, 10)

            node = _Node(self)
            rclpy.spin(node)
        except Exception as e:  # noqa: BLE001
            print(f"Falha ao iniciar subscriber ROS2 (/person_found): {e}")


def _ensure_drone_light(drone_prim_path: str = "/World/quadrotor"):
    stage = get_current_stage()
    light_path = f"{drone_prim_path}/person_light"
    light = UsdLux.SphereLight.Define(stage, light_path)
    light.CreateRadiusAttr(0.05)
    light.CreateIntensityAttr(0.0)
    light.CreateColorAttr((1.0, 0.2, 0.2))
    return light

def _spawn_random_people(
    n_people: int = 20,
    min_distance_m: float = 10.0,
    area_half_extent_m: float = 60.0,
    area_center_xy: tuple[float, float] = (0.0, 0.0),
    z: float = 0.0,
):
    assets_root = get_assets_root_path()
    if not assets_root:
        print("Não consegui localizar os assets do Isaac Sim.")
        return

    asset_rel_path = "People/Characters/original_male_adult_police_04/male_adult_police_04.usd"

    min_d_requested = max(0.0, float(min_distance_m))
    half = max(1.0, float(area_half_extent_m))
    cx = float(area_center_xy[0])
    cy = float(area_center_xy[1])
    z = float(z)

    print(
        "[people] spawn bounds: "
        f"x=[{(cx - half):.2f},{(cx + half):.2f}] "
        f"y=[{(cy - half):.2f},{(cy + half):.2f}] "
        f"min_distance={min_d_requested:.2f}m count={int(n_people)}"
    )

    def _sample_positions(min_d: float) -> list[tuple[float, float, float]]:
        positions: list[tuple[float, float, float]] = []
        max_attempts = max(4000, n_people * 800)
        attempts = 0
        while len(positions) < n_people and attempts < max_attempts:
            attempts += 1
            x = random.uniform(cx - half, cx + half)
            y = random.uniform(cy - half, cy + half)

            ok = True
            if min_d > 0.0:
                for (px, py, _pz) in positions:
                    if math.hypot(x - px, y - py) < min_d:
                        ok = False
                        break

            if ok:
                positions.append((float(x), float(y), float(z)))
        return positions

    min_d = float(min_d_requested)
    positions = _sample_positions(min_d)
    while len(positions) < n_people and min_d > 0.0:
        min_d = max(0.0, min_d * 0.9)
        positions = _sample_positions(min_d)

    if len(positions) < n_people:
        # As a last resort, ignore spacing but keep everyone inside bounds.
        min_d = 0.0
        positions = _sample_positions(min_d)

    specs = []
    for i, p in enumerate(positions, start=1):
        specs.append(
            PersonSpawnSpec(
                prim_path=f"/World/People/person_{i}",
                asset_rel_path=asset_rel_path,
                position=p,
            )
        )

    n = spawn_people(assets_root, specs)
    if min_d_requested > 0.0 and min_d < min_d_requested:
        print(f"{n} pessoas adicionadas. (min_distance relaxada: {min_d_requested:.2f} -> {min_d:.2f} m)")
    else:
        print(f"{n} pessoas adicionadas.")


async def main_async():
    config, project_root = load_config()

    print("=" * 60)
    print("Projeto Isaac Sim iniciado")
    print(f"Root do projeto: {project_root}")
    print(f"Config carregada: {config}")
    print("=" * 60)

    print(f"ISAAC_SIM_PATH={os.getenv('ISAAC_SIM_PATH')}")
    print(f"PEGASUS_PATH={os.getenv('PEGASUS_PATH')}")
    print(f"PX4_PATH={os.getenv('PX4_PATH')}")

    px4_path = os.getenv("PX4_PATH")

    if World.instance():
        World.instance().clear_instance()

    world = World(stage_units_in_meters=1.0)
    await world.initialize_simulation_context_async()
    world.scene.add_default_ground_plane()

    usd_cfg = config.get("world", {}).get("usd")
    if usd_cfg:
        usd_full_path = resolve_usd_path(project_root, usd_cfg)
        print(f"Tentando carregar USD: {usd_full_path}")

        if os.path.exists(usd_full_path):
            prim_path = "/World/Environment"
            if not is_prim_path_valid(prim_path):
                add_reference_to_stage(usd_path=usd_full_path, prim_path=prim_path)
                print(f"USD carregado em {prim_path}")
        else:
            print("USD não encontrado, seguindo só com o ground plane.")

    world_asset_rel = config.get("world", {}).get("isaac_asset_rel_path")
    if world_asset_rel:
        assets_root = get_assets_root_path()
        if not assets_root:
            print("Não consegui localizar os assets do Isaac Sim.")
        else:
            usd_path = f"{assets_root}/Isaac/{str(world_asset_rel).lstrip('/')}"
            prim_path = "/World/Environment"
            if not is_prim_path_valid(prim_path):
                add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
                print(f"Ambiente Isaac carregado em {prim_path}: {usd_path}")

    def _spawn_bounds_from_default_prim() -> tuple[tuple[float, float], float] | None:
        stage = get_current_stage()
        default_prim = stage.GetDefaultPrim()
        if not default_prim or not default_prim.IsValid():
            return None

        try:
            bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
            bbox = bbox_cache.ComputeWorldBound(default_prim)
            r = bbox.GetRange()
            minp = r.GetMin()
            maxp = r.GetMax()
            if not (minp and maxp):
                return None

            minx, miny = float(minp[0]), float(minp[1])
            maxx, maxy = float(maxp[0]), float(maxp[1])
            if not (math.isfinite(minx) and math.isfinite(miny) and math.isfinite(maxx) and math.isfinite(maxy)):
                return None

            cx = 0.5 * (minx + maxx)
            cy = 0.5 * (miny + maxy)
            half = 0.5 * min(maxx - minx, maxy - miny)
            if half <= 0.1:
                return None
            half *= 0.9

            return (cx, cy), float(half)
        except Exception:
            return None

    people_cfg = config.get("people", {}) or {}
    n_people = int(people_cfg.get("count", 20))
    min_distance_m = float(people_cfg.get("min_distance_m", 10.0))
    people_z = float(people_cfg.get("z", 0.0))
    world_cfg = config.get("world", {}) or {}
    area_half = float(world_cfg.get("spawn_area_half_extent_m", 60.0))
    area_center = world_cfg.get("spawn_area_center_xy", [0.0, 0.0])
    try:
        area_center_xy = (float(area_center[0]), float(area_center[1]))
    except Exception:
        area_center_xy = (0.0, 0.0)

    if bool(world_cfg.get("spawn_area_from_default_prim", False)):
        auto_bounds = _spawn_bounds_from_default_prim()
        if auto_bounds is not None:
            area_center_xy, area_half = auto_bounds
            print(f"[people] auto bounds from defaultPrim: center=({area_center_xy[0]:.2f},{area_center_xy[1]:.2f}) half={area_half:.2f}")
    _spawn_random_people(
        n_people=n_people,
        min_distance_m=min_distance_m,
        area_half_extent_m=area_half,
        area_center_xy=area_center_xy,
        z=people_z,
    )

    if px4_path:
        spawn_pegasus_quadrotor(world, px4_path, PegasusQuadrotorSpec())
        print("Drone adicionado com sucesso.")
        setup_sensors(config, "/World/quadrotor")
        light = _ensure_drone_light("/World/quadrotor")
        person_found_listener = _PersonFoundListener("/person_found")
        person_found_listener.start()
    else:
        print("PX4_PATH não definido; drone não foi criado.")
        light = None
        person_found_listener = None

    await world.reset_async()

    print("Cena pronta: até 1 drone (PX4) + 1 pessoa.")

    app = omni.kit.app.get_app()
    last_blink_t = time.monotonic()
    blink_on = False
    while app.is_running():
        world.step(render=True)

        if light is not None and person_found_listener is not None:
            found = person_found_listener.get()
            if found:
                now = time.monotonic()
                if now - last_blink_t >= 0.25:
                    blink_on = not blink_on
                    last_blink_t = now
                intensity = 2000.0 if blink_on else 0.0
            else:
                intensity = 0.0
                blink_on = False
                last_blink_t = time.monotonic()

            try:
                light.GetIntensityAttr().Set(float(intensity))
            except Exception:
                pass

        await app.next_update_async()


if __name__ == "__main__":
    run_coroutine(main_async())
