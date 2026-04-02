import os
import sys
from pathlib import Path

_STANDALONE_DIR = Path(__file__).resolve().parent
if str(_STANDALONE_DIR) not in sys.path:
    sys.path.insert(0, str(_STANDALONE_DIR))

import omni.kit.app
from omni.isaac.core import World
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.nucleus import get_assets_root_path
from omni.kit.async_engine import run_coroutine

from config_loader import load_config, resolve_usd_path
from spawn import PegasusQuadrotorSpec, PersonSpawnSpec, spawn_pegasus_quadrotor, spawn_person


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
    else:
        print("PX4_PATH não definido; drone não foi criado.")

    await world.reset_async()

    print("Cena pronta: até 1 drone (PX4) + 1 pessoa.")

    app = omni.kit.app.get_app()
    while app.is_running():
        await app.next_update_async()


if __name__ == "__main__":
    run_coroutine(main_async())
