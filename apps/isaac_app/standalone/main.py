from pathlib import Path
import os
import yaml

import omni.kit.app
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.kit.async_engine import run_coroutine


def load_config():
    project_root = Path(__file__).resolve().parents[3]
    config_path = project_root / "config" / "sim.yaml"
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f), project_root


def resolve_usd_path(project_root: Path, usd_path: str) -> str:
    return str((project_root / usd_path).resolve())


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

    await world.reset_async()

    print("Mundo iniciado com sucesso.")

    app = omni.kit.app.get_app()
    while app.is_running():
        await app.next_update_async()


if __name__ == "__main__":
    run_coroutine(main_async())