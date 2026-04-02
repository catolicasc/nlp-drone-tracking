from pathlib import Path
import yaml


def find_project_root(start: Path | None = None) -> Path:
    """
    Sobe diretórios até encontrar config/sim.yaml.
    Assim funciona quer o âncora seja .../standalone, .../standalone/main.py, etc.
    """
    d = (start or Path(__file__)).resolve()
    if d.is_file():
        d = d.parent
    for current in [d, *d.parents]:
        if (current / "config" / "sim.yaml").is_file():
            return current
    raise FileNotFoundError(
        f"Não encontrei config/sim.yaml acima de {d} (verifique se está a correr dentro do repo meu-projeto)."
    )


def load_config(here: Path | None = None):
    root = find_project_root(here)
    config_path = root / "config" / "sim.yaml"
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f), root


def resolve_usd_path(project_root: Path, usd_path: str) -> str:
    return str((project_root / usd_path).resolve())
