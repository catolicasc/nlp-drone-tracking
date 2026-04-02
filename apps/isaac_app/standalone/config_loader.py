from pathlib import Path
import yaml


def project_root_from_here(here: Path) -> Path:
    """Raiz do repositório: .../apps/isaac_app/standalone -> 3 níveis acima."""
    return here.resolve().parents[3]


def load_config(here: Path | None = None):
    if here is None:
        here = Path(__file__).resolve().parent
    root = project_root_from_here(here)
    config_path = root / "config" / "sim.yaml"
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f), root


def resolve_usd_path(project_root: Path, usd_path: str) -> str:
    return str((project_root / usd_path).resolve())
