import os
import sys
from pathlib import Path
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
from pxr import UsdLux

from config_loader import load_config, resolve_usd_path
from sensors import setup_sensors
from spawn import PegasusQuadrotorSpec, PersonSpawnSpec, spawn_pegasus_quadrotor, spawn_person


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


def _spawn_default_person():
    assets_root = get_assets_root_path()
    if not assets_root:
        print("Não consegui localizar os assets do Isaac Sim.")
        return
    spec = PersonSpawnSpec(
        prim_path="/World/People/person_1",
        asset_rel_path="People/Characters/original_male_adult_police_04/male_adult_police_04.usd",
        position=(4.0, 0.0, 0.0),
    )
    if spawn_person(assets_root, spec):
        print("1 pessoa adicionada.")


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

    _spawn_default_person()

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
