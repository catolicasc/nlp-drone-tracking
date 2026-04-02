from pathlib import Path

def test_project_structure():
    root = Path(__file__).resolve().parents[1]
    assert (root / "config" / "sim.yaml").exists()
    assert (root / "apps" / "isaac_app" / "standalone" / "main.py").exists()
