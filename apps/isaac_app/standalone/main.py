#!/usr/bin/env python
"""
Oracle Vision — standalone Isaac Sim app (padrão Pegasus).

Bootstrap via SimulationApp; lógica de simulação na classe OracleVisionApp.
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
if _pegasus_path and _pegasus_path not in sys.path:
    sys.path.insert(0, _pegasus_path)

import carb
from isaacsim import SimulationApp

_HEADLESS = os.getenv("HEADLESS", "0").strip().lower() in {"1", "true", "yes"}
simulation_app = SimulationApp(
    {
        "headless": _HEADLESS,
        "extra_args": ["--enable", "isaacsim.ros2.bridge"],
    }
)

import omni.timeline
from omni.isaac.core import World
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
from omni.isaac.nucleus import get_assets_root_path
from pxr import Usd, UsdGeom, UsdLux

from config_loader import load_config, resolve_usd_path
from sensors import setup_sensors
from spawn import PegasusQuadrotorSpec, PersonSpawnSpec, spawn_pegasus_quadrotor, spawn_people


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
    """App standalone: World + cena + drone Pegasus/PX4 + sensores + callbacks."""

    def __init__(self) -> None:
        self.config, self.project_root = load_config()
        self.timeline = omni.timeline.get_timeline_interface()
        self.stop_sim = False
        self._sim_started = False

        self._light = None
        self._person_found_listener: PersonFoundListener | None = None
        self._last_blink_t = time.monotonic()
        self._blink_on = False

        self._print_banner()

        if World.instance():
            World.instance().clear_instance()

        self.world = World(stage_units_in_meters=1.0)
        self._setup_world()
        self._setup_people()
        self._setup_drone_and_sensors()
        self._register_callbacks()
        self.world.reset()

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
        self.world.scene.add_default_ground_plane()
        self._load_usd_scene()
        self._load_isaac_environment()

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
        if not is_prim_path_valid(prim_path):
            add_reference_to_stage(usd_path=usd_full_path, prim_path=prim_path)
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
        if not is_prim_path_valid(prim_path):
            add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            print(f"Ambiente Isaac carregado em {prim_path}: {usd_path}")

    def _spawn_bounds_from_default_prim(self) -> tuple[tuple[float, float], float] | None:
        stage = get_current_stage()
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

        asset_rel_path = "People/Characters/original_male_adult_police_04/male_adult_police_04.usd"
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
            self.world,
            px4_path,
            PegasusQuadrotorSpec(init_pos=drone_pos),
        )
        print("Drone adicionado com sucesso.")
        setup_sensors(self.config, "/World/quadrotor")
        self._light = self._ensure_drone_light("/World/quadrotor")
        self._person_found_listener = PersonFoundListener("/person_found")
        self._person_found_listener.start()

    def _ensure_drone_light(self, drone_prim_path: str):
        stage = get_current_stage()
        light_path = f"{drone_prim_path}/person_light"
        light = UsdLux.SphereLight.Define(stage, light_path)
        light.CreateRadiusAttr(0.05)
        light.CreateIntensityAttr(0.0)
        light.CreateColorAttr((1.0, 0.2, 0.2))
        return light

    def _register_callbacks(self) -> None:
        self.world.add_render_callback("oracle_vision_drone_light", self._render_callback)
        self.world.add_timeline_callback("oracle_vision_timeline", self._timeline_callback)

    def _render_callback(self, _data) -> None:
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

    def _timeline_callback(self, _timeline_event) -> None:
        if self._sim_started and self.world.is_stopped():
            self.stop_sim = True

    def run(self) -> None:
        self.stop_sim = False
        self.timeline.play()
        self._sim_started = True

        while simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)

        carb.log_warn("Oracle Vision simulation app is closing.")
        self.timeline.stop()
        simulation_app.close()


def main() -> None:
    OracleVisionApp().run()


if __name__ == "__main__":
    main()
